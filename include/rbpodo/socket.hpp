/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#pragma once

#include <optional>
#include <string>

#if defined(_WIN32)
#include <WS2tcpip.h>
#elif defined(__linux__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace rb::podo {

class Socket {
 public:
  Socket(const std::string& address, int port);

  ~Socket();

  std::optional<std::string> recv() const;  // NOLINT

  bool send(std::string msg) const;  // NOLINT

 private:
#if defined(_WIN32)
  WSADATA wsa_data_{};
  SOCKET sock_{};
#elif defined(__linux__)
  int sock_{};
#endif
};

}  // namespace rb::podo