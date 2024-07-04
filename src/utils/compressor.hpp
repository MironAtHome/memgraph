// Copyright 2024 Memgraph Ltd.
//
// Use of this software is governed by the Business Source License
// included in the file licenses/BSL.txt; by using this file, you agree to be bound by the terms of the Business Source
// License, and you may not use this file except in compliance with the Business Source License.
//
// As of the Change Date specified in that file, in accordance with
// the Business Source License, use of this software will be governed
// by the Apache License, Version 2.0, included in the file
// licenses/APL.txt.

#pragma once

#include <sys/types.h>
#include <zlib.h>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <vector>
#include "spdlog/spdlog.h"

#define ZLIB_HEADER 0x78
#define ZLIB_LOW_COMPRESSION 0x01
#define ZLIB_FAST_COMPRESSION 0x5E
#define ZLIB_DEFAULT_COMPRESSION 0x9C
#define ZLIB_BEST_COMPRESSION 0xDA

namespace memgraph::utils {

struct DataBuffer {
  uint8_t *data{nullptr};
  uint32_t compressed_size = 0;
  uint32_t original_size = 0;

  // Default constructor
  DataBuffer() = default;

  // Destructor
  ~DataBuffer() { delete[] data; }

  // Copy constructor
  DataBuffer(const DataBuffer &other) = delete;

  // Copy assignment operator
  DataBuffer &operator=(const DataBuffer &other) = delete;

  // Move constructor
  DataBuffer(DataBuffer &&other) noexcept
      : data(other.data), compressed_size(other.compressed_size), original_size(other.original_size) {
    other.data = nullptr;
    other.compressed_size = 0;
    other.original_size = 0;
    spdlog::debug("DataBuffer move constructor");
    spdlog::debug("DataBuffer move constructor data: {}",
                  std::string_view(reinterpret_cast<char *>(data), original_size));
    spdlog::debug("DataBuffer move constructor compressed_size: {}", compressed_size);
    spdlog::debug("DataBuffer move constructor original_size: {}", original_size);
  }

  // Move assignment operator
  DataBuffer &operator=(DataBuffer &&other) noexcept {
    if (this != &other) {
      // Clean up existing data
      delete[] data;

      data = other.data;
      compressed_size = other.compressed_size;
      original_size = other.original_size;

      other.data = nullptr;
      other.compressed_size = 0;
      other.original_size = 0;
    }
    return *this;
  }
};

class Compressor {
 public:
  Compressor() = default;
  virtual ~Compressor() {}

  Compressor(const Compressor &) = default;
  Compressor &operator=(const Compressor &) = default;
  Compressor(Compressor &&) = default;
  Compressor &operator=(Compressor &&) = default;

  virtual DataBuffer Compress(uint8_t *input, uint32_t original_size) = 0;

  virtual DataBuffer Decompress(uint8_t *compressed_data, uint32_t compressed_size, uint32_t original_size) = 0;

  virtual bool IsCompressed(uint8_t *data, uint32_t size) const = 0;
};

class ZlibCompressor : public Compressor {
 protected:
  ZlibCompressor() = default;

  static ZlibCompressor *instance_;

 public:
  static ZlibCompressor *GetInstance();

  void operator=(const ZlibCompressor &) = delete;

  ZlibCompressor(const ZlibCompressor &) = delete;
  ZlibCompressor(ZlibCompressor &&) = delete;
  ZlibCompressor &operator=(ZlibCompressor &&) = delete;

  DataBuffer Compress(uint8_t *input, uint32_t original_size) override;

  DataBuffer Decompress(uint8_t *compressed_data, uint32_t compressed_size, uint32_t original_size) override;

  bool IsCompressed(uint8_t *data, uint32_t size) const override;
};

}  // namespace memgraph::utils
