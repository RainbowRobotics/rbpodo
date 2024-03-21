/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#pragma once

#include <optional>
#include <string>

#include "common.hpp"
#include "data_type.hpp"
#include "socket.hpp"

namespace rb::podo {

class CobotData {
 public:
  /**
   * Construct 'CobotData' instance with URI
   *
   * @param cb_address
   * @param data_port a port number for data channel (default: 5001)
   */
  explicit CobotData(const std::string& address, int port = kDataPort);

  ~CobotData();

  /// `CobotData` is not copyable
  CobotData(const CobotData&) = delete;

  /// `CobotData` is not copyable
  CobotData& operator=(const CobotData&) = delete;

  /// `CobotData` is movable
  CobotData(CobotData&&) noexcept = default;

  /// `CobotData` is movable
  CobotData& operator=(CobotData&&) = default;

  /**
   * Request data to control box.
   *
   * @param timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @return
   */
  [[nodiscard]] std::optional<SystemState> request_data(double timeout = -1.) const;

 protected:
  [[nodiscard]] virtual bool is_alive() const;

 private:
  Socket sock_;
};

}  // namespace rb::podo