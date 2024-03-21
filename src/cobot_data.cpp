/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#include <thread>

#include "rbpodo/cobot_data.hpp"

using namespace std::chrono_literals;

namespace {
inline unsigned min_(unsigned a, unsigned b) {
  if (a < b)
    return a;
  return b;
}
}  // namespace

namespace rb::podo {

CobotData::CobotData(const std::string& address, int port) : sock_(address, port) {}

CobotData::~CobotData() = default;

std::optional<SystemState> CobotData::request_data(double timeout) const {
  if (!sock_.send(kRequestDataCommand)) {
    return {};
  }

  auto to = Timeout(timeout);
  std::string recv_msg;
  while (is_alive()) {
    auto res = sock_.recv();
    if (res.has_value()) {
      if (res.value()[0] == '$' && res.value()[3] == 3) {
        recv_msg = res.value();
        break;
      }
    }
    if (to.is_elapsed()) {
      return {};
    }
    std::this_thread::sleep_for(10us);
  }

  auto len = ((unsigned long)recv_msg[2] << 8) | ((unsigned long)recv_msg[1]);
  SystemState state;
  memcpy(&state, &recv_msg[0], min_(sizeof(SystemState), len + 4));
  return state;
}

bool CobotData::is_alive() const {
  return true;
}

}  // namespace rb::podo