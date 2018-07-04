#pragma once

#include <map>
#include <new>
#include <string>

#include <glog/logging.h>

#include "communication/bolt/v1/codes.hpp"
#include "communication/bolt/v1/decoder/decoded_value.hpp"
#include "communication/bolt/v1/state.hpp"
#include "database/graph_db.hpp"
#include "query/exceptions.hpp"
#include "query/typed_value.hpp"
#include "utils/exceptions.hpp"

namespace communication::bolt {

template <typename TSession>
State HandleRun(TSession &session, State state, Marker marker) {
  const std::map<std::string, query::TypedValue> kEmptyFields = {
      {"fields", std::vector<query::TypedValue>{}}};

  if (marker != Marker::TinyStruct2) {
    DLOG(WARNING) << fmt::format(
        "Expected TinyStruct2 marker, but received 0x{:02X}!",
        utils::UnderlyingCast(marker));
    return State::Close;
  }

  DecodedValue query, params;
  if (!session.decoder_.ReadValue(&query, DecodedValue::Type::String)) {
    DLOG(WARNING) << "Couldn't read query string!";
    return State::Close;
  }

  if (!session.decoder_.ReadValue(&params, DecodedValue::Type::Map)) {
    DLOG(WARNING) << "Couldn't read parameters!";
    return State::Close;
  }

  if (state == State::WaitForRollback) {
    if (query.ValueString() == "ROLLBACK") {
      session.Abort();
      // One MessageSuccess for RUN command should be flushed.
      session.encoder_.MessageSuccess(kEmptyFields);
      // One for PULL_ALL should be chunked.
      session.encoder_.MessageSuccess({}, false);
      return State::Result;
    }
    DLOG(WARNING) << "Expected RUN \"ROLLBACK\" not received!";
    // Client could potentially recover if we move to error state, but we don't
    // implement rollback of single command in transaction, only rollback of
    // whole transaction so we can't continue in this transaction if we receive
    // new RUN command.
    return State::Close;
  }

  if (state != State::Idle) {
    // Client could potentially recover if we move to error state, but there is
    // no legitimate situation in which well working client would end up in this
    // situation.
    DLOG(WARNING) << "Unexpected RUN command!";
    return State::Close;
  }

  DCHECK(!session.encoder_buffer_.HasData())
      << "There should be no data to write in this state";

  DLOG(INFO) << fmt::format("[Run] '{}'", query.ValueString());
  bool in_explicit_transaction = false;
  if (session.db_accessor_) {
    // Transaction already exists.
    in_explicit_transaction = true;
  } else {
    // TODO: Possible (but very unlikely) race condition, where we have alive
    // session during shutdown, but is_accepting_transactions_ isn't yet false.
    // We should probably create transactions under some locking mechanism.
    if (!session.db_.is_accepting_transactions()) {
      // Db is shutting down and doesn't accept new transactions so we should
      // close this session.
      return State::Close;
    }
    // Create new transaction.
    session.db_accessor_ =
        std::make_unique<database::GraphDbAccessor>(session.db_);
  }

  // If there was not explicitly started transaction before maybe we are
  // starting one now.
  if (!in_explicit_transaction && query.ValueString() == "BEGIN") {
    // Check if query string is "BEGIN". If it is then we should start
    // transaction and wait for in-transaction queries.
    // TODO: "BEGIN" is not defined by bolt protocol or opencypher so we should
    // test if all drivers really denote transaction start with "BEGIN" string.
    // Same goes for "ROLLBACK" and "COMMIT".
    //
    // One MessageSuccess for RUN command should be flushed.
    session.encoder_.MessageSuccess(kEmptyFields);
    // One for PULL_ALL should be chunked.
    session.encoder_.MessageSuccess({}, false);
    return State::Result;
  }

  try {
    // This check is within try block because AdvanceCommand can throw.
    if (in_explicit_transaction) {
      if (query.ValueString() == "COMMIT") {
        session.Commit();
        // One MessageSuccess for RUN command should be flushed.
        session.encoder_.MessageSuccess(kEmptyFields);
        // One for PULL_ALL should be chunked.
        session.encoder_.MessageSuccess({}, false);
        return State::Result;
      } else if (query.ValueString() == "ROLLBACK") {
        session.Abort();
        // One MessageSuccess for RUN command should be flushed.
        session.encoder_.MessageSuccess(kEmptyFields);
        // One for PULL_ALL should be chunked.
        session.encoder_.MessageSuccess({}, false);
        return State::Result;
      }
      session.db_accessor_->AdvanceCommand();
    }

    auto &params_map = params.ValueMap();
    std::map<std::string, query::TypedValue> params_tv(params_map.begin(),
                                                       params_map.end());
    session
        .interpreter_(query.ValueString(), *session.db_accessor_, params_tv,
                      in_explicit_transaction)
        .PullAll(session.result_stream_);

    if (!in_explicit_transaction) {
      session.Commit();
    }
    // The query engine has already stored all query data in the buffer.
    // We should only send the first chunk now which is the success
    // message which contains header data. The rest of this data (records
    // and summary) will be sent after a PULL_ALL command from the client.
    if (!session.encoder_buffer_.FlushFirstChunk()) {
      DLOG(WARNING) << "Couldn't flush header data from the buffer!";
      return State::Close;
    }
    return State::Result;
  } catch (const std::exception &e) {
    // clear header success message
    session.encoder_buffer_.Clear();
    DLOG(WARNING) << fmt::format("Error message: {}", e.what());
    if (const auto *p = dynamic_cast<const utils::StacktraceException *>(&e)) {
      DLOG(WARNING) << fmt::format("Error trace: {}", p->trace());
    }

    auto code_message = [&e]() -> std::pair<std::string, std::string> {
      if (dynamic_cast<const query::QueryException *>(&e)) {
        // Clients expect 4 strings separated by dots. First being database name
        // (for example: Neo, Memgraph...), second being either ClientError,
        // TransientError or DatabaseError (or ClientNotification for warnings).
        // ClientError means wrong query, do not retry. DatabaseError means
        // something wrong in database, do not retry. TransientError means query
        // failed, but if retried it may succeed, retry it.
        //
        // Third and fourth strings being namespace and specific error name.
        // It is not really important what we put there since we don't expect
        // any special handling of specific exceptions on client side, but we
        // need to make sure that we don't accidentally return some exception
        // name which clients handle in a special way. For example, if client
        // receives *.TransientError.Transaction.Terminate it will not rerun
        // query even though TransientError was returned, because of Neo's
        // semantics of that error.
        //
        // QueryException was thrown, only changing the query or existing
        // database data could make this query successful. Return ClientError to
        // discourage retry of this query.
        return {"Memgraph.ClientError.MemgraphError.MemgraphError", e.what()};
      }
      if (dynamic_cast<const utils::BasicException *>(&e)) {
        // Exception not derived from QueryException was thrown which means that
        // database probably aborted transaction because of some timeout,
        // deadlock, serialization error or something similar. We return
        // TransientError since retry of same transaction could succeed.
        return {"Memgraph.TransientError.MemgraphError.MemgraphError",
                e.what()};
      }
      if (dynamic_cast<const std::bad_alloc *>(&e)) {
        // std::bad_alloc was thrown, God knows in which state is database ->
        // terminate.
        LOG(FATAL) << "Memgraph is out of memory";
      }
      // All exceptions used in memgraph are derived from BasicException. Since
      // we caught some other exception we don't know what is going on. Return
      // DatabaseError, log real message and return generic string.
      LOG(ERROR) << "Unknown exception occurred during query execution "
                 << e.what();
      return {"Memgraph.DatabaseError.MemgraphError.MemgraphError",
              "An unknown exception occurred, this is unexpected. Real message "
              "should be in database logs."};
    }();

    bool fail_sent = session.encoder_.MessageFailure(
        {{"code", code_message.first}, {"message", code_message.second}});
    if (!fail_sent) {
      DLOG(WARNING) << "Couldn't send failure message!";
      return State::Close;
    }
    if (!in_explicit_transaction) {
      session.Abort();
      return State::ErrorIdle;
    }
    return State::ErrorWaitForRollback;
  }
}

// TODO: Get rid of duplications in PullAll/DiscardAll functions.
template <typename Session>
State HandlePullAll(Session &session, State state, Marker marker) {
  if (marker != Marker::TinyStruct) {
    DLOG(WARNING) << fmt::format(
        "Expected TinyStruct marker, but received 0x{:02X}!",
        utils::UnderlyingCast(marker));
    return State::Close;
  }

  if (state != State::Result) {
    DLOG(WARNING) << "Unexpected PULL_ALL!";
    // Same as `unexpected RUN` case.
    return State::Close;
  }
  // Flush pending data to the client, the success message is streamed
  // from the query engine, it contains statistics from the query run.
  if (!session.encoder_buffer_.Flush()) {
    DLOG(WARNING) << "Couldn't flush data from the buffer!";
    return State::Close;
  }
  return State::Idle;
}

template <typename Session>
State HandleDiscardAll(Session &session, State state, Marker marker) {
  if (marker != Marker::TinyStruct) {
    DLOG(WARNING) << fmt::format(
        "Expected TinyStruct marker, but received 0x{:02X}!",
        utils::UnderlyingCast(marker));
    return State::Close;
  }

  if (state != State::Result) {
    DLOG(WARNING) << "Unexpected DISCARD_ALL!";
    // Same as `unexpected RUN` case.
    return State::Close;
  }
  // Clear all pending data and send a success message.
  session.encoder_buffer_.Clear();
  if (!session.encoder_.MessageSuccess()) {
    DLOG(WARNING) << "Couldn't send success message!";
    return State::Close;
  }
  return State::Idle;
}

template <typename Session>
State HandleReset(Session &session, State, Marker marker) {
  // IMPORTANT: This implementation of the Bolt RESET command isn't fully
  // compliant to the protocol definition. In the protocol it is defined
  // that this command should immediately stop any running commands and
  // reset the session to a clean state. That means that we should always
  // make a look-ahead for the RESET command before processing anything.
  // Our implementation, for now, does everything in a blocking fashion
  // so we cannot simply "kill" a transaction while it is running. So
  // now this command only resets the session to a clean state. It
  // does not IGNORE running and pending commands as it should.
  if (marker != Marker::TinyStruct) {
    DLOG(WARNING) << fmt::format(
        "Expected TinyStruct marker, but received 0x{:02X}!",
        utils::UnderlyingCast(marker));
    return State::Close;
  }
  // clear all pending data and send a success message
  session.encoder_buffer_.Clear();
  if (!session.encoder_.MessageSuccess()) {
    DLOG(WARNING) << "Couldn't send success message!";
    return State::Close;
  }
  if (session.db_accessor_) {
    session.Abort();
  }
  return State::Idle;
}

/**
 * Executor state run function
 * This function executes an initialized Bolt session.
 * It executes: RUN, PULL_ALL, DISCARD_ALL & RESET.
 * @param session the session that should be used for the run
 */
template <typename Session>
State StateExecutingRun(Session &session, State state) {
  Marker marker;
  Signature signature;
  if (!session.decoder_.ReadMessageHeader(&signature, &marker)) {
    DLOG(WARNING) << "Missing header data!";
    return State::Close;
  }

  if (signature == Signature::Run) {
    return HandleRun(session, state, marker);
  } else if (signature == Signature::PullAll) {
    return HandlePullAll(session, state, marker);
  } else if (signature == Signature::DiscardAll) {
    return HandleDiscardAll(session, state, marker);
  } else if (signature == Signature::Reset) {
    return HandleReset(session, state, marker);
  } else {
    DLOG(WARNING) << fmt::format("Unrecognized signature recieved (0x{:02X})!",
                                 utils::UnderlyingCast(signature));
    return State::Close;
  }
}
}  // namespace communication::bolt
