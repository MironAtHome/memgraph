#pragma once

#include <netdb.h>
#include <cstring>

#include "io/network/network_error.hpp"
#include "utils/underlying_cast.hpp"

namespace io {

class AddrInfo {
  AddrInfo(struct addrinfo* info) : info(info) {}

 public:
  ~AddrInfo() { freeaddrinfo(info); }

  static AddrInfo get(const char* addr, const char* port) {
    struct addrinfo hints;
    memset(&hints, 0, sizeof(struct addrinfo));

    hints.ai_family = AF_UNSPEC;      // IPv4 and IPv6
    hints.ai_socktype = SOCK_STREAM;  // TCP socket
    hints.ai_flags = AI_PASSIVE;

    struct addrinfo* result;
    auto status = getaddrinfo(addr, port, &hints, &result);

    if (status != 0) throw NetworkError(gai_strerror(status));

    return AddrInfo(result);
  }

  operator struct addrinfo*() { return info; }

 private:
  struct addrinfo* info;
};
}
