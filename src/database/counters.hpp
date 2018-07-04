#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "data_structures/concurrent/concurrent_map.hpp"

namespace database {

/** A set of counter that are guaranteed to produce unique, consecutive values
 * on each call. */
class Counters {
 public:
  virtual ~Counters() {}

  /**
   * Returns the current value of the counter with the given name, and
   * increments that counter. If the counter with the given name does not exist,
   * a new counter is created and this function returns 0.
   */
  virtual int64_t Get(const std::string &name) = 0;

  /**
   * Sets the counter with the given name to the given value. Returns nothing.
   * If the counter with the given name does not exist, a new counter is created
   * and set to the given value.
   */
  virtual void Set(const std::string &name, int64_t values) = 0;
};

/** Implementation for the single-node memgraph */
class SingleNodeCounters : public Counters {
 public:
  int64_t Get(const std::string &name) override;
  void Set(const std::string &name, int64_t value) override;

 private:
  ConcurrentMap<std::string, std::atomic<int64_t>> counters_;
};

}  // namespace database
