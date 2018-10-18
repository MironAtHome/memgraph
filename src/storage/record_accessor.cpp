#include "glog/logging.h"

#include "database/graph_db_accessor.hpp"
#include "database/state_delta.hpp"
#include "query/exceptions.hpp"
#include "storage/edge.hpp"
#include "storage/record_accessor.hpp"
#include "storage/vertex.hpp"
#include "utils/thread/sync.hpp"

using database::StateDelta;

template <typename TRecord>
RecordAccessor<TRecord>::RecordAccessor(AddressT address,
                                        database::GraphDbAccessor &db_accessor)
    : db_accessor_(&db_accessor),
      address_(db_accessor.db().storage().LocalizedAddressIfPossible(address)) {
}

template <typename TRecord>
PropertyValue RecordAccessor<TRecord>::PropsAt(storage::Property key) const {
  return current().properties_.at(key);
}

template <>
void RecordAccessor<Vertex>::PropsSet(storage::Property key,
                                      PropertyValue value) {
  auto &dba = db_accessor();
  auto delta = StateDelta::PropsSetVertex(dba.transaction_id(), gid(), key,
                                          dba.PropertyName(key), value);
  update().properties_.set(key, value);
  if (is_local()) {
    dba.UpdatePropertyIndex(key, *this, &update());
  }
  ProcessDelta(delta);
}

template <>
void RecordAccessor<Edge>::PropsSet(storage::Property key,
                                    PropertyValue value) {
  auto &dba = db_accessor();
  auto delta = StateDelta::PropsSetEdge(dba.transaction_id(), gid(), key,
                                        dba.PropertyName(key), value);

  update().properties_.set(key, value);
  ProcessDelta(delta);
}

template <>
void RecordAccessor<Vertex>::PropsErase(storage::Property key) {
  auto &dba = db_accessor();
  auto delta =
      StateDelta::PropsSetVertex(dba.transaction_id(), gid(), key,
                                 dba.PropertyName(key), PropertyValue::Null);
  update().properties_.set(key, PropertyValue::Null);
  ProcessDelta(delta);
}

template <>
void RecordAccessor<Edge>::PropsErase(storage::Property key) {
  auto &dba = db_accessor();
  auto delta =
      StateDelta::PropsSetEdge(dba.transaction_id(), gid(), key,
                               dba.PropertyName(key), PropertyValue::Null);
  update().properties_.set(key, PropertyValue::Null);
  ProcessDelta(delta);
}

template <typename TRecord>
void RecordAccessor<TRecord>::PropsClear() {
  std::vector<storage::Property> to_remove;
  for (const auto &kv : update().properties_) to_remove.emplace_back(kv.first);
  for (const auto &prop : to_remove) {
    PropsErase(prop);
  }
}

template <typename TRecord>
const PropertyValueStore &RecordAccessor<TRecord>::Properties() const {
  return current().properties_;
}

template <typename TRecord>
bool RecordAccessor<TRecord>::operator==(const RecordAccessor &other) const {
  DCHECK(db_accessor_->transaction_id() == other.db_accessor_->transaction_id())
      << "Not in the same transaction.";
  return address_ == other.address_;
}

template <typename TRecord>
database::GraphDbAccessor &RecordAccessor<TRecord>::db_accessor() const {
  return *db_accessor_;
}

template <typename TRecord>
gid::Gid RecordAccessor<TRecord>::gid() const {
  return is_local() ? address_.local()->gid_ : address_.gid();
}

template <typename TRecord>
typename RecordAccessor<TRecord>::AddressT RecordAccessor<TRecord>::address()
    const {
  return address_;
}

template <typename TRecord>
typename RecordAccessor<TRecord>::AddressT
RecordAccessor<TRecord>::GlobalAddress() const {
  return is_local() ? storage::Address<mvcc::VersionList<TRecord>>(
                          gid(), db_accessor_->db_.WorkerId())
                    : address_;
}

template <typename TRecord>
RecordAccessor<TRecord> &RecordAccessor<TRecord>::SwitchNew() {
  if (is_local()) {
    if (!new_) {
      // if new_ is not set yet, look for it
      // we can just Reconstruct the pointers, old_ will get initialized
      // to the same value as it has now, and the amount of work is the
      // same as just looking for a new_ record
      if (!Reconstruct())
        DLOG(FATAL)
            << "RecordAccessor::SwitchNew - accessor invalid after Reconstruct";
    }
  } else {
    // A remote record only sees local updates, until the command is advanced.
    // So this does nothing, as the old/new switch happens below.
  }
  current_ = new_ ? new_ : old_;
  return *this;
}

template <typename TRecord>
RecordAccessor<TRecord> &RecordAccessor<TRecord>::SwitchOld() {
  current_ = old_ ? old_ : new_;
  return *this;
}

template <typename TRecord>
bool RecordAccessor<TRecord>::Reconstruct() const {
  auto &dba = db_accessor();
  address_.local()->find_set_old_new(dba.transaction(), old_, new_);
  current_ = old_ ? old_ : new_;
  return old_ != nullptr || new_ != nullptr;
}

template <typename TRecord>
TRecord &RecordAccessor<TRecord>::update() const {
  auto &dba = db_accessor();
  // Edges have lazily initialize mutable, versioned data (properties).
  if (std::is_same<TRecord, Edge>::value && current_ == nullptr) {
    bool reconstructed = Reconstruct();
    DCHECK(reconstructed) << "Unable to initialize record";
  }

  const auto &t = dba.transaction();
  if (!new_ && old_->is_expired_by(t))
    throw RecordDeletedError();
  else if (new_ && new_->is_expired_by(t))
    throw RecordDeletedError();

  if (new_) return *new_;

  new_ = address_.local()->update(t);

  DCHECK(new_ != nullptr) << "RecordAccessor.new_ is null after update";
  return *new_;
}

template <typename TRecord>
const TRecord &RecordAccessor<TRecord>::current() const {
  // Edges have lazily initialize mutable, versioned data (properties).
  if (std::is_same<TRecord, Edge>::value && current_ == nullptr)
    RecordAccessor::Reconstruct();
  DCHECK(current_ != nullptr) << "RecordAccessor.current_ pointer is nullptr";
  return *current_;
}

template <typename TRecord>
void RecordAccessor<TRecord>::ProcessDelta(
    const database::StateDelta &delta) const {
  db_accessor().wal().Emplace(delta);
}

template class RecordAccessor<Vertex>;
template class RecordAccessor<Edge>;
