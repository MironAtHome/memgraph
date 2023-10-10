// Copyright 2023 Memgraph Ltd.
//
// Use of this software is governed by the Business Source License
// included in the file licenses/BSL.txt; by using this file, you agree to be bound by the terms of the Business Source
// License, and you may not use this file except in compliance with the Business Source License.
//
// As of the Change Date specified in that file, in accordance with
// the Business Source License, use of this software will be governed
// by the Apache License, Version 2.0, included in the file
// licenses/APL.txt.

#include "memory_control.hpp"
#include <cstdint>
#include <iostream>
#include <sstream>
#include <thread>
#include "utils/logging.hpp"
#include "utils/memory_tracker.hpp"

#if USE_JEMALLOC
#include <jemalloc/jemalloc.h>
#endif

namespace memgraph::memory {

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define STRINGIFY_HELPER(x) #x
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define STRINGIFY(x) STRINGIFY_HELPER(x)

std::string get_thread_id() {
  std::ostringstream oss;
  oss << std::this_thread::get_id();
  return oss.str();
}

#if USE_JEMALLOC

static void *my_alloc(extent_hooks_t *extent_hooks, void *new_addr, size_t size, size_t alignment, bool *zero,
                      bool *commit, unsigned arena_ind);
static bool my_dalloc(extent_hooks_t *extent_hooks, void *addr, size_t size, bool committed, unsigned arena_ind);
static void my_destroy(extent_hooks_t *extent_hooks, void *addr, size_t size, bool committed, unsigned arena_ind);
static bool my_commit(extent_hooks_t *extent_hooks, void *addr, size_t size, size_t offset, size_t length,
                      unsigned arena_ind);
static bool my_decommit(extent_hooks_t *extent_hooks, void *addr, size_t size, size_t offset, size_t length,
                        unsigned arena_ind);
static bool my_purge_forced(extent_hooks_t *extent_hooks, void *addr, size_t size, size_t offset, size_t length,
                            unsigned arena_ind);
extent_hooks_t *old_hooks = nullptr;

static extent_hooks_t custom_hooks = {
    .alloc = &my_alloc,
    .dalloc = &my_dalloc,
    .destroy = &my_destroy,
    .commit = &my_commit,
    .decommit = &my_decommit,
    .purge_lazy = nullptr,
    .purge_forced = &my_purge_forced,
    .split = nullptr,
    .merge = nullptr,
};

static const extent_hooks_t *new_hooks = &custom_hooks;

void *my_alloc(extent_hooks_t *extent_hooks, void *new_addr, size_t size, size_t alignment, bool *zero, bool *commit,
               unsigned arena_ind) {
  // This needs to be before, to throw exception in case of too big alloc
  if (*commit) [[likely]] {
    memgraph::utils::total_memory_tracker.Alloc(static_cast<int64_t>(size));
    if (arena_tracking[arena_ind]) [[unlikely]] {
      std::string thread_id = get_thread_id();
      std::cout << "aloc:" << thread_id << std::endl;
      std::cout << "thread id to transaction id" << thread_id_to_transaction_id[get_thread_id()] << std::endl;
      transaction_id_tracker[thread_id_to_transaction_id[get_thread_id()]].Alloc(static_cast<int64_t>(size));
    }
  }

  auto *ptr = old_hooks->alloc(extent_hooks, new_addr, size, alignment, zero, commit, arena_ind);
  if (ptr == nullptr) [[unlikely]] {
    if (*commit) {
      memgraph::utils::total_memory_tracker.Free(static_cast<int64_t>(size));
      if (arena_tracking[arena_ind]) [[unlikely]] {
        transaction_id_tracker[thread_id_to_transaction_id[get_thread_id()]].Free(static_cast<int64_t>(size));
      }
    }
    return ptr;
  }

  return ptr;
}

static bool my_dalloc(extent_hooks_t *extent_hooks, void *addr, size_t size, bool committed, unsigned arena_ind) {
  auto err = old_hooks->dalloc(extent_hooks, addr, size, committed, arena_ind);

  if (err) [[unlikely]] {
    return err;
  }

  if (committed) [[likely]] {
    memgraph::utils::total_memory_tracker.Free(static_cast<int64_t>(size));

    if (arena_tracking[arena_ind]) [[unlikely]] {
      transaction_id_tracker[thread_id_to_transaction_id[get_thread_id()]].Free(static_cast<int64_t>(size));
    }
  }

  return false;
}

static void my_destroy(extent_hooks_t *extent_hooks, void *addr, size_t size, bool committed, unsigned arena_ind) {
  if (committed) [[likely]] {
    memgraph::utils::total_memory_tracker.Free(static_cast<int64_t>(size));
    if (arena_tracking[arena_ind]) [[unlikely]] {
      transaction_id_tracker[thread_id_to_transaction_id[get_thread_id()]].Free(static_cast<int64_t>(size));
    }
  }

  old_hooks->destroy(extent_hooks, addr, size, committed, arena_ind);
}

static bool my_commit(extent_hooks_t *extent_hooks, void *addr, size_t size, size_t offset, size_t length,
                      unsigned arena_ind) {
  auto err = old_hooks->commit(extent_hooks, addr, size, offset, length, arena_ind);

  if (err) {
    return err;
  }

  memgraph::utils::total_memory_tracker.Alloc(static_cast<int64_t>(length));
  if (arena_tracking[arena_ind]) [[unlikely]] {
    transaction_id_tracker[thread_id_to_transaction_id[get_thread_id()]].Alloc(static_cast<int64_t>(size));
  }

  return false;
}

static bool my_decommit(extent_hooks_t *extent_hooks, void *addr, size_t size, size_t offset, size_t length,
                        unsigned arena_ind) {
  MG_ASSERT(old_hooks && old_hooks->decommit);
  auto err = old_hooks->decommit(extent_hooks, addr, size, offset, length, arena_ind);

  if (err) {
    return err;
  }

  memgraph::utils::total_memory_tracker.Free(static_cast<int64_t>(length));
  if (arena_tracking[arena_ind]) [[unlikely]] {
    transaction_id_tracker[thread_id_to_transaction_id[get_thread_id()]].Free(static_cast<int64_t>(size));
  }

  return false;
}

static bool my_purge_forced(extent_hooks_t *extent_hooks, void *addr, size_t size, size_t offset, size_t length,
                            unsigned arena_ind) {
  MG_ASSERT(old_hooks && old_hooks->purge_forced);
  auto err = old_hooks->purge_forced(extent_hooks, addr, size, offset, length, arena_ind);

  if (err) [[unlikely]] {
    return err;
  }
  memgraph::utils::total_memory_tracker.Free(static_cast<int64_t>(length));

  if (arena_tracking[arena_ind]) [[unlikely]] {
    transaction_id_tracker[thread_id_to_transaction_id[get_thread_id()]].Free(static_cast<int64_t>(size));
  }

  return false;
}

#endif

void SetHooks() {
#if USE_JEMALLOC

  uint64_t allocated{0};
  uint64_t sz{sizeof(allocated)};

  sz = sizeof(unsigned);
  unsigned n_arenas{0};
  int err = mallctl("opt.narenas", (void *)&n_arenas, &sz, nullptr, 0);

  if (err) {
    return;
  }

  spdlog::trace("n areanas {}", n_arenas);

  if (nullptr != old_hooks) {
    return;
  }

  for (int i = 0; i < n_arenas; i++) {
    arena_tracking[i] = 0;
    std::string func_name = "arena." + std::to_string(i) + ".extent_hooks";

    size_t hooks_len = sizeof(old_hooks);

    int err = mallctl(func_name.c_str(), &old_hooks, &hooks_len, nullptr, 0);

    if (err) {
      LOG_FATAL("Error getting hooks for jemalloc arena {}", i);
    }

    // Due to the way jemalloc works, we need first to set their hooks
    // which will trigger creating arena, then we can set our custom hook wrappers

    err = mallctl(func_name.c_str(), nullptr, nullptr, &old_hooks, sizeof(old_hooks));

    MG_ASSERT(old_hooks);
    MG_ASSERT(old_hooks->alloc);
    MG_ASSERT(old_hooks->dalloc);
    MG_ASSERT(old_hooks->destroy);
    MG_ASSERT(old_hooks->commit);
    MG_ASSERT(old_hooks->decommit);
    MG_ASSERT(old_hooks->purge_forced);
    MG_ASSERT(old_hooks->purge_lazy);
    MG_ASSERT(old_hooks->split);
    MG_ASSERT(old_hooks->merge);

    custom_hooks.purge_lazy = old_hooks->purge_lazy;
    custom_hooks.split = old_hooks->split;
    custom_hooks.merge = old_hooks->merge;

    if (err) {
      LOG_FATAL("Error setting jemalloc hooks for jemalloc arena {}", i);
    }

    err = mallctl(func_name.c_str(), nullptr, nullptr, &new_hooks, sizeof(new_hooks));

    if (err) {
      LOG_FATAL("Error setting custom hooks for jemalloc arena {}", i);
    }
  }

#endif
}

unsigned GetArenaForThread() {
#if USE_JEMALLOC
  unsigned thread_arena{0};
  size_t size_thread_arena = sizeof(thread_arena);
  int err = mallctl("thread.arena", &thread_arena, &size_thread_arena, nullptr, 0);
  if (err) {
    return -1;
  }
  return thread_arena;
#endif
  return -1;
}

bool AddTrackingOnArena(unsigned arena_id) {
#if USE_JEMALLOC
  arena_tracking[arena_id].fetch_add(1);
#endif
  return false;
}

bool RemoveTrackingOnArena(unsigned arena_id) {
#if USE_JEMALLOC
  arena_tracking[arena_id].fetch_sub(1);
#endif
  return true;
}

void UpdateThreadToTransactionId(const std::thread::id &thread_id, uint64_t transaction_id) {
  std::ostringstream oss;
  oss << thread_id;
  thread_id_to_transaction_id[oss.str()] = transaction_id;
  std::cout << "set:" << oss.str() << std::endl;
}

void ResetThreadToTransactionId(const std::thread::id &thread_id) {
  std::ostringstream oss;
  oss << thread_id;
  thread_id_to_transaction_id.erase(oss.str());
}

void PurgeUnusedMemory() {
#if USE_JEMALLOC
  mallctl("arena." STRINGIFY(MALLCTL_ARENAS_ALL) ".purge", nullptr, nullptr, nullptr, 0);
#endif
}

#undef STRINGIFY
#undef STRINGIFY_HELPER

}  // namespace memgraph::memory
