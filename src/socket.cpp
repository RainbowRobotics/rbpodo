/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#include "rbpodo/socket.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>

namespace rb::podo {

Socket::Socket(const std::string& address, int port) {
#if defined(_WIN32)
  int startup_result = WSAStartup(MAKEWORD(2, 2), &wsa_data_);
  if (startup_result) {
    std::stringstream ss;
    ss << "Cannot start WinSock (error: " << startup_result << ")";
    throw std::runtime_error(ss.str());
  }
#elif defined(__linux__)
#endif

  sock_ = socket(AF_INET, SOCK_STREAM, 0);
#if defined(_WIN32)
  if (sock_ == INVALID_SOCKET) {
    WSACleanup();
#elif defined(__linux__)
  if (sock_ < 0) {
#endif
    throw std::runtime_error("cannot create socket");
  }

  struct sockaddr_in server_address {};

  server_address.sin_family = AF_INET;
  if (inet_pton(AF_INET, address.c_str(), &server_address.sin_addr) != 1) {
#if defined(_WIN32)
    closesocket(sock_);
    WSACleanup();
#elif defined(__linux__)
    close(sock_);
#endif
    throw std::runtime_error("network address conversion failed");
  }
  server_address.sin_port = htons(port);

  if (connect(sock_, (struct sockaddr*)&server_address, sizeof(server_address)) != 0) {
#if defined(_WIN32)
    closesocket(sock_);
    WSACleanup();
#elif defined(__linux__)
    close(sock_);
#endif
    throw std::runtime_error("connection failed");
  }

#if defined(_WIN32)
  u_long mode = 1;
  if (ioctlsocket(sock_, FIONBIO, &mode) != NO_ERROR) {
    closesocket(sock_);
    WSACleanup();
#elif defined(__linux__)
  if (fcntl(sock_, F_SETFL, fcntl(sock_, F_GETFL) | O_NONBLOCK) < 0) {
    close(sock_);
#endif
    throw std::runtime_error("failed to pu the socket in non-blocking mode");
  }

  int opt = 1;
  setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, (const char*)&opt, sizeof(opt));
}

Socket::~Socket() {
#if defined(_WIN32)
  closesocket(sock_);
  WSACleanup();
#elif defined(__linux__)
  close(sock_);
#endif
}

std::optional<std::string> Socket::recv() const {
  static const size_t kBufferSize = 1024;
  char buf[kBufferSize];

  long res;
  if ((res = ::recv(sock_, &buf[0], kBufferSize, 0)) < 0) {
    return {};
  }
  return std::string(&buf[0], res);
}

bool Socket::send(std::string msg) const {
  msg += "\n";
  if (::send(sock_, msg.data(), (int)msg.size(), 0) < 0) {
    return false;
  }
  return true;
}

}  // namespace rb::podo