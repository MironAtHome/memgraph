#include <limits>
#include <mutex>

#include "glog/logging.h"

#include "database/state_delta.hpp"
#include "transactions/engine_single_node.hpp"

namespace tx {

SingleNodeEngine::SingleNodeEngine(durability::WriteAheadLog *wal)
    : wal_(wal) {}

Transaction *SingleNodeEngine::Begin() {
  VLOG(11) << "[Tx] Starting transaction " << counter_ + 1;
  std::lock_guard<SpinLock> guard(lock_);

  TransactionId id{++counter_};
  auto t = new Transaction(id, active_, *this);
  active_.insert(id);
  store_.emplace(id, t);
  if (wal_) {
    wal_->Emplace(database::StateDelta::TxBegin(id));
  }
  return t;
}

CommandId SingleNodeEngine::Advance(TransactionId id) {
  std::lock_guard<SpinLock> guard(lock_);

  auto it = store_.find(id);
  DCHECK(it != store_.end())
      << "Transaction::advance on non-existing transaction";

  Transaction *t = it->second.get();
  if (t->cid_ == std::numeric_limits<CommandId>::max())
    throw TransactionError(
        "Reached maximum number of commands in this "
        "transaction.");

  return ++(t->cid_);
}

CommandId SingleNodeEngine::UpdateCommand(TransactionId id) {
  std::lock_guard<SpinLock> guard(lock_);
  auto it = store_.find(id);
  DCHECK(it != store_.end())
      << "Transaction::advance on non-existing transaction";
  return it->second->cid_;
}

void SingleNodeEngine::Commit(const Transaction &t) {
  VLOG(11) << "[Tx] Commiting transaction " << t.id_;
  std::lock_guard<SpinLock> guard(lock_);
  clog_.set_committed(t.id_);
  active_.remove(t.id_);
  if (wal_) {
    wal_->Emplace(database::StateDelta::TxCommit(t.id_));
  }
  store_.erase(store_.find(t.id_));
}

void SingleNodeEngine::Abort(const Transaction &t) {
  VLOG(11) << "[Tx] Aborting transaction " << t.id_;
  std::lock_guard<SpinLock> guard(lock_);
  clog_.set_aborted(t.id_);
  active_.remove(t.id_);
  if (wal_) {
    wal_->Emplace(database::StateDelta::TxAbort(t.id_));
  }
  store_.erase(store_.find(t.id_));
}

CommitLog::Info SingleNodeEngine::Info(TransactionId tx) const {
  return clog_.fetch_info(tx);
}

Snapshot SingleNodeEngine::GlobalGcSnapshot() {
  std::lock_guard<SpinLock> guard(lock_);

  // No active transactions.
  if (active_.size() == 0) {
    auto snapshot_copy = active_;
    snapshot_copy.insert(counter_ + 1);
    return snapshot_copy;
  }

  // There are active transactions.
  auto snapshot_copy = store_.find(active_.front())->second->snapshot();
  snapshot_copy.insert(active_.front());
  return snapshot_copy;
}

Snapshot SingleNodeEngine::GlobalActiveTransactions() {
  std::lock_guard<SpinLock> guard(lock_);
  Snapshot active_transactions = active_;
  return active_transactions;
}

TransactionId SingleNodeEngine::LocalLast() const {
  std::lock_guard<SpinLock> guard(lock_);
  return counter_;
}

TransactionId SingleNodeEngine::GlobalLast() const { return LocalLast(); }

TransactionId SingleNodeEngine::LocalOldestActive() const {
  std::lock_guard<SpinLock> guard(lock_);
  return active_.empty() ? counter_ + 1 : active_.front();
}

void SingleNodeEngine::GarbageCollectCommitLog(TransactionId tx_id) {
  clog_.garbage_collect_older(tx_id);
}

void SingleNodeEngine::LocalForEachActiveTransaction(
    std::function<void(Transaction &)> f) {
  std::lock_guard<SpinLock> guard(lock_);
  for (auto transaction : active_) {
    f(*store_.find(transaction)->second);
  }
}

Transaction *SingleNodeEngine::RunningTransaction(TransactionId tx_id) {
  std::lock_guard<SpinLock> guard(lock_);
  auto found = store_.find(tx_id);
  CHECK(found != store_.end())
      << "Can't return snapshot for an inactive transaction";
  return found->second.get();
}

void SingleNodeEngine::EnsureNextIdGreater(TransactionId tx_id) {
  std::lock_guard<SpinLock> guard(lock_);
  counter_ = std::max(tx_id, counter_);
}

}  // namespace tx
