/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

/**
* @file cobot.hpp
* @brief Contains definitions of Cobot, Response and ResponseCollector classes.
*
* This file contains the definitions of the Cobot class, Response class and
* ResponseCollector class. These classes are used to interact with the RB Cobot via
* data channel.
*/

#pragma once

#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <ostream>
#include <queue>
#include <regex>
#include <string>
#include <thread>

#include "common.hpp"
#include "data_type.hpp"
#include "error_code.hpp"
#include "socket.hpp"

namespace rb::podo {

/**
* @class Response
* @brief A class to represent the response of an operation or command.
*/
class Response {
 public:
  /// The possible types of a Response.
  enum class Type { ACK, Info, Warn, Error, Unknown };

  explicit Response(std::string raw);

  /// Parses the raw response.
  void parse();

  /// Returns the raw response message.
  [[nodiscard]] const std::string& raw() const { return raw_; }

  /// Returns the type of this Response.
  [[nodiscard]] Type type() const { return type_; }

  /// Returns the category of this Response.
  [[nodiscard]] const std::string& category() const { return category_; }

  /// Returns the message of this Response.
  [[nodiscard]] const std::string& msg() const { return msg_; }

  /// Returns a string representation of this Response.
  [[nodiscard]] std::string str() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
  }

  friend std::ostream& operator<<(std::ostream& out, const Response& res);

 private:
  std::string raw_;

  Type type_{};
  std::string category_{};
  std::string msg_{};
};

/**
* @class ResponseCollector
* @brief A collection of Response.
*/
class ResponseCollector : public std::deque<Response> {
 public:
  enum ConfigFlag : unsigned int {
    /// When the `wait_until function` is called, the past (remaining in the ResponseCollector) responses are also examined.
    EnableCheckOldResponses = 1 << 0,

    /// Ack messages are automatically removed from the ResponseCollector after message inspection (wait_until).
    RemoveAckAutomatically = 1 << 1,

    Default = RemoveAckAutomatically
  };

  explicit ResponseCollector(unsigned int flag = ConfigFlag::Default);

  // TODO: Need to check copy and move operation whether member variables are copied and moved, respectively.

  template <typename Func>
  ResponseCollector filter(Func pred, bool remain = false) {
    static_assert(std::is_same_v<std::invoke_result_t<Func, const Response&>, bool>);
    ResponseCollector res(flag_);
    if (cb_) {
      res.set_callback(cb_);
    }
    if (remain) {
      std::copy_if(begin(), end(), std::back_inserter(res), pred);
    } else {
      std::copy_if(begin(), end(), std::back_inserter(res), pred);
      erase(std::remove_if(begin(), end(), pred), end());
    }
    return res;
  }

  ResponseCollector type_filter(Response::Type type, bool remain = false);

  void swap_responses(ResponseCollector& response_collector);

  void clear();

  void add(Response&& response);

  void add(const Response& response);

  void set_callback(const std::function<void(const Response&)>& cb);

  void clear_callback();

  ResponseCollector info(bool remain = false);

  ResponseCollector warn(bool remain = false);

  ResponseCollector error(bool remain = false);

  ResponseCollector ack(bool remain = false);

  ResponseCollector category_filter(const std::string& filter, bool remain = false);

  [[nodiscard]] bool has_error() const;

  ResponseCollector& throw_if_not_empty() {
    if (!empty()) {
      std::stringstream ss;
      ss << *this;
      throw std::runtime_error("Not empty: " + ss.str());
    }
    return *this;
  }

  [[nodiscard]] unsigned int flag() const { return flag_; }

  void set_flag(unsigned int flag) { flag_ = flag; }

  friend std::ostream& operator<<(std::ostream& out, const ResponseCollector& response_collector);

 private:
  unsigned int flag_{ConfigFlag::Default};
  std::function<void(const Response&)> cb_{nullptr};
};

/**
 * @class ReturnType
 * @brief Return type of Cobot class functions
 */
class ReturnType {
 public:
  enum Type { Undefined, Success, Timeout, Error };

  ReturnType(Type type = Undefined, double remain_time = 0.) : type_(type), remain_time_(remain_time) {}

  [[nodiscard]] Type type() const { return type_; }

  [[nodiscard]] double remain_time() const { return remain_time_; }

  void set_remain_time(double remain_time) { remain_time_ = remain_time; }

  [[nodiscard]] bool is_success() const { return type_ == Success; }

  [[nodiscard]] bool is_timeout() const { return type_ == Timeout; }

  [[nodiscard]] bool is_error() const { return type_ == Error; }

  friend std::ostream& operator<<(std::ostream& out, const ReturnType& ret);

 private:
  Type type_;
  double remain_time_;
};

/**
* @class Cobot
* @brief Class to interact with the 'Cobot'.
*/
template <typename Type = StandardVector>
class Cobot {
 public:
  using Joint = typename Type::Joint;
  using Point = typename Type::Point;
  using JointRef = typename Type::JointRef;
  using PointRef = typename Type::PointRef;
  using JointConstRef = typename Type::JointConstRef;
  using PointConstRef = typename Type::PointConstRef;

  /**
  * Construct 'Cobot' instance with URI (Control box)
  *
  * @param address
  * @param port a port number for command channel (default: 5000)
  */
  explicit Cobot(const std::string& address, int port = kCommandPort)
      : address_(address), port_(port), sock_(address, port) {}

  ~Cobot() = default;

  /// `Cobot` is not copyable
  Cobot(const Cobot&) = delete;

  /// `Cobot` is not copyable
  Cobot& operator=(const Cobot&) = delete;

  /// `Cobot` is movable
  Cobot(Cobot&&) noexcept = default;

  /// `Cobot` is movable
  Cobot& operator=(Cobot&&) = default;

  [[nodiscard]] std::string address() const { return address_; }

  [[nodiscard]] int port() const { return port_; }

  /**
   * Retrieves information about the control box.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[out] info Information including "system version" and "robot box type"
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType get_control_box_info(ResponseCollector& response_collector, ControlBoxInfo& info, double timeout = -1.,
                                  bool return_on_error = false) {
    std::stringstream ss;
    ss << "get_control_box_info()";
    sock_.send(ss.str());
    auto res = wait_until_ack_message(response_collector, timeout, return_on_error);
    if (res.is_success()) {
      const auto& check = [=](const Response& res) {
        return res.type() == Response::Type::Info && res.category() == "controlbox";
      };
      if ((res = wait_until(response_collector, check, res.remain_time(), return_on_error)).is_success()) {
        std::stringstream msg(response_collector.back().msg());
        char c;
        msg >> info.system_version >> c >> info.robot_box_type;
      }
      return res;
    } else {
      return res;
    }
  }

  /**
   * Retrieves whether the robot is in the Idle or Moving state.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[out] robot_state Object storing whether the robot is Idle or Moving
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType get_robot_state(ResponseCollector& response_collector, RobotState& robot_state, double timeout = -1.,
                             bool return_on_error = false) {
    double out;
    auto ret = get_system_variable(response_collector, SystemVariable::SD_ROBOT_STATE, out, timeout, return_on_error);
    if (std::abs(out - 1.) < std::numeric_limits<double>::epsilon()) {
      robot_state = RobotState::Idle;
    } else if (std::abs(out - 3.) < std::numeric_limits<double>::epsilon()) {
      robot_state = RobotState::Moving;
    } else {
      robot_state = RobotState::Unknown;
    }
    return ret;
  }

  /**
   * Activate the robot.
   * If the robot is already activated or has some errors, it returns immediately.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType activate(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = true) {
    double state_info;
    auto res = get_system_variable(response_collector, SystemVariable::SD_INIT_STATE_INFO, state_info, timeout,
                                   return_on_error);
    if (!res.is_success()) {
      return res;
    }
    if ((((int)state_info) & 0x3f) == 6) {
      return res;
    }

    std::stringstream ss;
    ss << "mc jall init";
    sock_.send(ss.str());
    res = wait_until_ack_message(response_collector, timeout, return_on_error);
    if (res.is_success()) {
      const auto& check = [=](const Response& res) {
        return res.type() == Response::Type::Info && res.msg() == "done";
      };
      return wait_until(response_collector, check, res.remain_time(), return_on_error);
    } else {
      return res;
    }
  }

  /**
   * Shutdown the robot.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType shutdown(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = true) {
    std::stringstream ss;
    ss << "AvadaKedavra()";
    sock_.send(ss.str());
    auto res = wait_until_ack_message(response_collector, timeout, return_on_error);
    if (res.is_success()) {
      const auto& check = [=](const Response& res) {
        return res.type() == Response::Type::Error && res.msg() == "0";
      };
      res = wait_until(response_collector, check, res.remain_time(), false);
      if (res.is_success()) {
        response_collector.pop_back();
      }
      return res;
    } else {
      return res;
    }
  }

  ReturnType get_system_variable(ResponseCollector& response_collector, SystemVariable sv, double& out,
                                 double timeout = -1., bool return_on_error = false) {
    const auto& id = randstr();
    std::stringstream ss;
    ss << "print(" << to_string(sv) << ",\"" << id << R"([", "]"))";
    sock_.send(ss.str());
    auto res = wait_until_ack_message(response_collector, timeout, return_on_error);
    if (res.is_success()) {
      return wait_for_printed_value(response_collector, out, std::regex(id + "\\[(.*)\\]"), res.remain_time(),
                                    return_on_error);
    } else {
      return res;
    }
  }

  ReturnType print_variable(ResponseCollector& response_collector, const std::string& variable_name, std::string& out,
                            double timeout = -1., bool return_on_error = false) {
    const auto& id = randstr();
    std::stringstream ss;
    ss << "print(" << variable_name << ",\"" << id << R"([", "]"))";
    sock_.send(ss.str());
    auto res = wait_until_ack_message(response_collector, timeout, return_on_error);
    if (res.is_success()) {
      return wait_for_printed_value(response_collector, out, std::regex(id + "\\[(.*)\\]"), res.remain_time(),
                                    return_on_error);
    } else {
      return res;
    }
  }

  /**
   * This function returns the TCP information of the current robot.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[out] point Returns the TCP of the current robot based on the global coordinate system. (Unit: mm & degree)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType get_tcp_info(ResponseCollector& response_collector, PointRef point, double timeout = -1.,
                          bool return_on_error = false) {
    const auto& id = randstr();
    std::stringstream ss;
    ss << "print(get_tcp_info(), \"" << id << R"([", "]"))";
    sock_.send(ss.str());
    auto res = wait_until_ack_message(response_collector, timeout, return_on_error);
    if (res.is_success()) {
      return wait_for_printed_value(response_collector, point, std::regex(id + "\\[(.*)\\]"), res.remain_time(),
                                    return_on_error);
    } else {
      return res;
    }
  }

  /**
   * This function returns the TFC (Tool flange center) information of the current robot.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[out] point Returns the TFC of the current robot based on the global coordinate system. (Unit: mm & degree)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType get_tfc_info(ResponseCollector& response_collector, PointRef point, double timeout = -1.,
                          bool return_on_error = false) {
    const auto& id = randstr();
    std::stringstream ss;
    ss << "print(get_tfc_info(), \"" << id << R"([", "]"))";
    sock_.send(ss.str());
    auto res = wait_until_ack_message(response_collector, timeout, return_on_error);
    if (res.is_success()) {
      return wait_for_printed_value(response_collector, point, std::regex(id + "\\[(.*)\\]"), res.remain_time(),
                                    return_on_error);
    } else {
      return res;
    }
  }

  /**
   * Set the tool payload w.r.t. the manufacturer’s default tool coordinate system.
   *
   * @warning The value set in this function returns to the default value after the program ends.
   * If this function is not called in program-flow, the value set in the Setup page is used.
   * During program flow, the value set in this function is maintained until this function is called again.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] weight payload weight (Unit: Kg)
   * @param[in] com_x payload center of mass x-axis value with respect to the manufacturer's default coordinate system. (Unit: mm)
   * @param[in] com_y payload center of mass y-axis value with respect to the manufacturer's default coordinate system. (Unit: mm)
   * @param[in] com_z payload center of mass z-axis value with respect to the manufacturer's default coordinate system. (Unit: mm)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_payload_info(ResponseCollector& response_collector, double weight, double com_x, double com_y,
                              double com_z, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_payload_info(" << weight << "," << com_x << "," << com_y << "," << com_z << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Set the tool volume w.r.t. the manufacturer’s default tool coordinate system.
   *
   * @warning The value set in this function returns to the default value after the program ends.
   * If this function is not called in program-flow, the value set in the Setup page is used.
   * During program flow, the value set in this function is maintained until this function is called again.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] x_width width of tool along x-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
   * @param[in] y_width width of tool along y-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
   * @param[in] z_width width of tool along z-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
   * @param[in] x_offset offset of box along x-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
   * @param[in] y_offset offset of box along y-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
   * @param[in] z_offset offset of box along z-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_tool_box(ResponseCollector& response_collector, double x_width, double y_width, double z_width,
                              double x_offset, double y_offset, double z_offset, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "set rb_tool_box " << x_width << "," << y_width << "," << z_width << "," << x_offset << "," << y_offset << "," << z_offset;
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Set the TCP position and orientation w.r.t. the manufacturer’s default tool coordinate system.
   *
   * @warning The value set in this function returns to the default value after the program ends.
   * If this function is not called in program-flow, the value set in the Setup page is used.
   * During program flow, the value set in this function is maintained until this function is called again.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] point position and orientation of tcp with respect to manufacturer's default tool coordinate system. (x, y, z, rx, ry, rz) (Unit: mm & degree)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_tcp_info(ResponseCollector& response_collector, PointConstRef point, double timeout = -1.,
                          bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_tcp_info(" << point[0] << "," << point[1] << "," << point[2] << "," << point[3] << "," << point[4] << ","
       << point[5] << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Set user coordinate
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] id id of user coordinate to change (0~2)
   * @param[in] point position and orientation of coordinate with respect to base frame. (x, y, z, rx, ry, rz) (Unit: mm & degree)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_user_coordinate(ResponseCollector& response_collector, int id, PointConstRef point,
                                 double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "set rb_manual_user_coord_6d " << id << ",1," << point[0] << "," << point[1] << "," << point[2] << ","
       << point[3] << "," << point[4] << "," << point[5];
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType set_operation_mode(ResponseCollector& response_collector, OperationMode mode, double timeout = -1.,
                                bool return_on_error = false) {
    std::stringstream ss;
    ss << "pgmode " << (mode == OperationMode::Real ? "real" : "simulation");
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Set the overall speed control bar. (bottom speed control bar in UI).
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] speed Desired speed control bar position (0~1)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_speed_bar(ResponseCollector& response_collector, double speed, double timeout = -1.,
                           bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_speed_bar(" << speed << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType set_freedrive_mode(ResponseCollector& response_collector, bool on, double timeout = -1.,
                                bool return_on_error = false) {
    std::stringstream ss;
    if (on) {
      ss << "freedrive_teach_on()";
    } else {
      ss << "freedrive_teach_off()";
    }
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function turns on/off the collision detection function.
   *
   * @warning The value set in this function returns to the default value after the program ends.
   * If this function is not called in program-flow, the value set in the Setup page is used.
   * During program flow, the value set in this function is maintained until this function is called again.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] on The variable represents an on/off state, where 0 is off and 1 is on.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_collision_onoff(ResponseCollector& response_collector, bool on, double timeout = -1.,
                                 bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_collision_onoff(" << (on ? "1" : "0") << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Sets the collision sensitivity (threshold).
   *
   * @warning The value set in this function returns to the default value after the program ends.
   * If this function is not called in program-flow, the value set in the Setup page is used.
   * During program flow, the value set in this function is maintained until this function is called again.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] threshold The variable represents an threshold value. The lower the value, the more sensitive to collision. (0~1)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_collision_threshold(ResponseCollector& response_collector, double threshold, double timeout = -1.,
                                     bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_collision_th(" << threshold << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Set the stop-mode after the collision detection.
   *
   * @warning The value set in this function returns to the default value after the program ends.
   * If this function is not called in program-flow, the value set in the Setup page is used.
   * During program flow, the value set in this function is maintained until this function is called again.
   *
   * @example robot.set_collision_mode(rc, CollisionMode::EvasionStop) // After detecting a collision, the robot moves a
   * little in the direction to avoid external force and then stops the movement.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] mode Stop mode. (CollisionMode)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_collision_mode(ResponseCollector& response_collector, CollisionMode mode, double timeout = -1.,
                                bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_collision_mode(" << (int)mode << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Set the program flow direction after the collision detection.
   *
   * @warning The value set in this function returns to the default value after the program ends.
   * If this function is not called in program-flow, the value set in the Setup page is used.
   * During program flow, the value set in this function is maintained until this function is called again.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] mode Stop mode. (CollisionReactionMode)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_collision_after(ResponseCollector& response_collector, CollisionReactionMode mode,
                                 double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_collision_after(" << (int)mode << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Sets the overall speed (velocity) multiplier.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] multiplier Multiply variable. (0~2) Default value is 1.0.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_speed_multiplier(ResponseCollector& response_collector, double multiplier, double timeout = -1.,
                                  bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_speed_multiplier(" << multiplier << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Sets the overall acceleration multiplier.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] multiplier Multiply variable. (0~2) Default value is 1.0.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_acc_multiplier(ResponseCollector& response_collector, double multiplier, double timeout = -1.,
                                bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_acc_multiplier(" << multiplier << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Sets fixed joint velocity/acceleration for J-series motions (MoveJ, MoveJB, MoveJL).
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] speed Speed/velocity (Unit: deg/s). Does not use negative value.
   * @param[in] acceleration Acceleration (Unit: deg/s^2). Does not use negative value.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_speed_acc_j(ResponseCollector& response_collector, double speed, double acceleration,
                             double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_speed_acc_j(" << speed << "," << acceleration << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Sets fixed linear velocity/acceleration for L-series motions (MoveL, MovePB, MoveLB, MoveITPL).
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] speed Speed/velocity (Unit: mm/s)
   * @param[in] acceleration Acceleration (Unit: mm/s^2)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_speed_acc_l(ResponseCollector& response_collector, double speed, double acceleration,
                             double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_speed_acc_l(" << speed << "," << acceleration << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Set the digital output of the control box.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] port Port number for the digital output. (0~15)
   * @param[in] mode Output mode selection (DigitalIOMode)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_box_dout(ResponseCollector& response_collector, int port, DigitalIOMode mode, double timeout = -1.,
                          bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_box_dout(" << port << "," << (int)mode << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Set the analog output of the control box.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] port Port number for the analog output. (0~15)
   * @param[in] voltage Desired output voltage (0~10V, Unit: V)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_box_aout(ResponseCollector& response_collector, int port, double voltage, double timeout = -1.,
                          bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_box_aout(" << port << "," << voltage << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Toggles the current digital output of the control box.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] port Port number for the digital output. (0~15)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_box_dout_toggle(ResponseCollector& response_collector, int port, double timeout = -1.,
                                 bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_box_dout_toggle(" << port << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType set_dout_bit_combination(ResponseCollector& response_collector, int first_port, int last_port,
                                      unsigned int value, Endian mode, double timeout = -1.,
                                      bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_dout_bit_combination(" << first_port << "," << last_port << "," << value << "," << (int)mode << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Set the serial communication (RS232/485) provided by the Tool Flange of the robot arm.
   *
   * @warning The value set in this function returns to the default value after the program ends.
   * If this function is not called in program-flow, the value set in the Setup page is used.
   * During program flow, the value set in this function is maintained until this function is called again.
   *
   * @example robot.set_serial_tool(rc, 115200, 1, 0) // Set tool-flange serial comm. : baud rate = 115200 / stop bit = 1 / parity = none
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] baud_rate Communication speed (Baud rate)
   * @param[in] stop_bit Stop bit (0 or 1, Default value is 1)
   * @param[in] parity_bit Parity bit (0: none, 1: odd, 2: even, Default value is 0)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_serial_tool(ResponseCollector& response_collector, int baud_rate, int stop_bit, int parity_bit,
                             double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_serial_tool(" << baud_rate << "," << stop_bit << "," << parity_bit << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Set the serial communication (RS232/485) provided by the control box.
   *
   * @warning The value set in this function returns to the default value after the program ends.
   * If this function is not called in program-flow, the value set in the Setup page is used.
   * During program flow, the value set in this function is maintained until this function is called again.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] baud_rate Communication speed (Baud rate)
   * @param[in] stop_bit Stop bit (0 or 1, Default value is 1)
   * @param[in] parity_bit Parity bit (0: none, 1: odd, 2: even, Default value is 0)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType set_serial_box(ResponseCollector& response_collector, int baud_rate, int stop_bit, int parity_bit,
                            double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "set_serial_box(" << baud_rate << "," << stop_bit << "," << parity_bit << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType calc_fk_tcp(ResponseCollector& response_collector, PointRef point, double j0, double j1, double j2,
                         double j3, double j4, double j5, double timeout = -1., bool return_on_error = false) {
    const auto& id = randstr();
    std::stringstream ss;
    ss << "print(calc_fk_tcp(" << j0 << "," << j1 << "," << j2 << "," << j3 << "," << j4 << "," << j5 << "), \"" << id
       << R"([", "]"))";
    sock_.send(ss.str());
    auto res = wait_until_ack_message(response_collector, timeout, return_on_error);
    if (res.is_success()) {
      return wait_for_printed_value(response_collector, point, std::regex(id + "\\[(.*)\\]"), res.remain_time(),
                                    return_on_error);
    } else {
      return res;
    }
  }

  /**
   * Calculate forward kinematics with given joint position.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[out] point A variable will store the pose of the end-effector (TCP) in given joint position
   * @param[in] joint Joint position
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType calc_fk_tcp(ResponseCollector& response_collector, PointRef point, JointConstRef joint,
                         double timeout = -1.,  // NOLINT
                         bool return_on_error = false) {
    return calc_fk_tcp(response_collector, point, joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], timeout,
                       return_on_error);
  }

  /**
   * A function that makes TCP to move in a straight line to the target point.
   *
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] point Target TCP pose. (Point)
   * @param[in] speed Speed (Unit: mm/s)
   * @param[in] acceleration Acceleration (Unit: mm/s^2)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_l(ResponseCollector& response_collector, PointConstRef point, double speed, double acceleration,
                    double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_l(" << Type::point_to_string(point) << "," << speed << "," << acceleration << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * A function that makes TCP to move in a straight line to the target point.
   * Enter the target point as a value relative to the current TCP value.
   *
   * @example robot.move_l_rel(rc, {0, 100, -200, 0, 0, 0}, 300, 400, 0) // move TCP (0,100,-200) w.r.t. Base coordinate (speed/acceleration = 300 / 400)
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] point Relative position & orientation value. (Point)
   * @param[in] speed Speed (unit: mm/s)
   * @param[in] acceleration Acceleration (unit: mm/s^2)
   * @param[in] frame Reference frame for the relative point value.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_l_rel(ResponseCollector& response_collector, PointConstRef point, double speed, double acceleration,
                        ReferenceFrame frame, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_l_rel(" << Type::point_to_string(point) << "," << speed << "," << acceleration << "," << (int)frame
       << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Move the robot arm to the target joint angle in joint space.
   *
   * @example robot.move_j(rc, {0, 0, 90, 0, 90, 0}, 60, 80) // move joint angles to (0,0,90,0,90,0) degree with speed/acceleration = 60/80.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] joint Target joint angles. (Joint)
   * @param[in] speed Speed (Unit: deg/s)
   * @param[in] acceleration Acceleration (Unit: deg/s^2)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_j(ResponseCollector& response_collector, JointConstRef joint, double speed, double acceleration,
                    double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_j(" << Type::joint_to_string(joint) << "," << speed << "," << acceleration << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function moves to the target point using the move_j method rather than a straight line.
   *
   * @example robot.move_jl(rc, {100, 200, 300, 0, 0, 0}, 20, 5) // Move TCP to '{100,200,300,0,0,0}' via MoveJ method.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] point Target TCP posture. (Point)
   * @param[in] speed Speed (Unit: deg/s)
   * @param[in] acceleration Acceleration (Unit: deg/s^2)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_jl(ResponseCollector& response_collector, PointConstRef point, double speed, double acceleration,
                     double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_jl(" << Type::point_to_string(point) << "," << speed << "," << acceleration << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Initialize (Clear) the point list to be used in MovePB.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_pb_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false) {
    sock_.send("move_pb_clear()");
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function adds the points used in MovePB to the list.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] point Target TCP posture. (Point)
   * @param[in] speed Speed (Unit: mm/s)
   * @param[in] option Blending option (0: blend based on ratio, 1: blend based on distance.)
   * @param[in] blending_value Blending value (0~1 in ratio option or distance in distance option (Unit: mm)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_pb_add(ResponseCollector& response_collector, PointConstRef point, double speed,
                         BlendingOption option, double blending_value, double timeout = -1.,
                         bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_pb_add(" << Type::point_to_string(point) << "," << speed << "," << (int)option << "," << blending_value
       << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function executes MovePB using the points added in move_pb_add.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] acceleration Acceleration (Unit: mm/s^2)
   * @param[in] option Orientation option (0: Intended, 1: Constant)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_pb_run(ResponseCollector& response_collector, double acceleration, MovePBOption option,
                         double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_pb_run(" << acceleration << "," << (int)option << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Initialize (Clear) the point list to be used in MoveITPL.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_itpl_clear(ResponseCollector& response_collector, double timeout = -1.,
                             bool return_on_error = false) {
    sock_.send("move_itpl_clear()");
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function adds the points used in MoveITPL to the list.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] point Target TCP posture. (Point)
   * @param[in] speed Speed (Unit: mm/s)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_itpl_add(ResponseCollector& response_collector, PointConstRef point, double speed,
                           double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_itpl_add(" << Type::point_to_string(point) << "," << speed << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function executes MoveITPL using the points added in move_itpl_add.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] acceleration Acceleration
   * @param[in] option Orientation/motion option. (CA : Combined Arc mode)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_itpl_run(ResponseCollector& response_collector, double acceleration, MoveITPLOption option,
                           double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_itpl_run(" << acceleration << "," << (int)option << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Initialize (Clear) the point list to be used in MoveLC.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_lc_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false) {
    sock_.send("move_lc_clear()");
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function adds the points used in MoveLC to the list.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] point Target TCP posture. (Point)
   * @param[in] speed Speed (Unit: mm/s)
   * @param[in] property 0 or 1 (0 : Pass through linear motion, 1 : Pass through circular motion)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_lc_add(ResponseCollector& response_collector, PointConstRef point, double speed,
                         MoveLCProperty property, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_lc_add(" << Type::point_to_string(point) << "," << speed << "," << (int)property << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function executes MoveITPL using the points added in move_lc_add.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] acceleration Acceleration
   * @param[in] option Orientation options
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_lc_run(ResponseCollector& response_collector, double acceleration, MoveLCOption option,
                         double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_lc_run(" << acceleration << "," << (int)option << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Initialize (Clear) the point list to be used in MoveLB.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_lb_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false) {
    sock_.send("move_lb_clear()");
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function adds the points used in MoveLB to the list.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] point Target TCP posture. (Point)
   * @param[in] blend_distance Blend distance. (Unit: mm)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_lb_add(ResponseCollector& response_collector, PointConstRef point, double blend_distance,
                         double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_lb_add(" << Type::point_to_string(point) << "," << blend_distance << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function executes MoveLB using the points added in move_lb_add.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] speed Speed (Unit: mm/s)
   * @param[in] acceleration Acceleration (Unit: mm/s^2)
   * @param[in] option Orientation options. (0 : Intended, 1 : Constant)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_lb_run(ResponseCollector& response_collector, double speed, double acceleration, MoveLBOption option,
                         double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_lb_run(" << speed << "," << acceleration << "," << (int)option << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function performs a movement that draws an arc through via & target points.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] via_point via Point TCP posture.
   * @param[in] target_point target Point TCP posture.
   * @param[in] speed Speed (Unit: mm/s)
   * @param[in] acceleration Acceleration (Unit: mm/s^2)
   * @param[in] option Orientation options. (0 : Intended, 1 : Constant, 2 : Radial, 3 : Smooth)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_c_points(ResponseCollector& response_collector, PointConstRef via_point, PointConstRef target_point,
                           double speed, double acceleration, MoveCOrientationOption option, double timeout = -1.,
                           bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_c_points(" << Type::point_to_string(via_point) << "," << Type::point_to_string(target_point) << ","
       << speed << "," << acceleration << "," << (int)option << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function performs a movement that draws an arc through via & target points.
   *
   * @example robot.move_c_axis(rc, {200, 200, 200, 0, 0, 0}, 1, 0, 0, 180, 50, 10, 2)
   * // Rotate 180 degrees around the x-axis. Center of rotation is '{200, 200, 200, 0, 0, 0}'. Based on the center point of the rotation, the orientation of the TCP is changed together.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] center Center of the rotation (Unit: mm)
   * @param[in] x_axis Rotation axis's x axis vector
   * @param[in] y_axis Rotation axis's y axis vector
   * @param[in] z_axis Rotation axis's z axis vector
   * @param[in] angle Rotation angle (Unit: deg)
   * @param[in] speed Speed (Unit: mm/s)
   * @param[in] acceleration Acceleration (Unit: mm/s^2)
   * @param[in] option Rotation options. (0 : Intended, 1 : Constant, 2 : Radial)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_c_axis(ResponseCollector& response_collector, PointConstRef center, double x_axis, double y_axis,
                         double z_axis, double angle, double speed, double acceleration, MoveCRotationOption option,
                         double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_c_axis(" << Type::point_to_string(center) << "," << x_axis << "," << y_axis << "," << z_axis << ","
       << angle << "," << speed << "," << acceleration << "," << (int)option << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * Initialize (Clear) the point list to be used in MoveJB.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_jb_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false) {
    sock_.send("move_jb_clear()");
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function adds the joint-angles used in MoveJB to the list.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] joint Target joint angles.
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_jb_add(ResponseCollector& response_collector, JointConstRef joint, double timeout = -1.,
                         bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_jb_add(" << Type::joint_to_string(joint) << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /**
   * This function executes MoveJB using the points added in move_jb_add.
   * 
   * @param[in] response_collector A collector object to accumulate and manage the response message.
   * @param[in] speed Speed (Unit: deg/s)
   * @param[in] acceleration Acceleration (Unit: deg/s^2)
   * @param[in] timeout The maximum duration (in seconds) to wait for a response before timing out.
   * @param[in] return_on_error A boolean flag indicating whether the function should immediately return upon encountering an error.
   * @return ReturnType
   */
  ReturnType move_jb_run(ResponseCollector& response_collector, double speed, double acceleration, double timeout = -1.,
                         bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_jb_run(" << speed << "," << acceleration << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType move_jb2_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false) {
    sock_.send("move_jb2_clear()");
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType move_jb2_add(ResponseCollector& response_collector, JointConstRef joint, double speed, double acceleration,
                          double blending_value, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_jb2_add(" << Type::joint_to_string(joint) << "," << speed << "," << acceleration << ",0,"
       << blending_value << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType move_jb2_run(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_jb2_run()";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType move_servo_j(ResponseCollector& response_collector, JointConstRef joint, double t1, double t2, double gain,
                          double alpha, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_servo_j(" << Type::joint_to_string(joint) << "," << t1 << "," << t2 << "," << gain << "," << alpha
       << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType move_servo_l(ResponseCollector& response_collector, PointConstRef point, double t1, double t2, double gain,
                          double alpha, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_servo_l(" << Type::point_to_string(point) << "," << t1 << "," << t2 << "," << gain << "," << alpha
       << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType move_servo_t(ResponseCollector& response_collector, JointConstRef joint, double t1, double t2,
                          int compensation, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_servo_t(" << Type::joint_to_string(joint) << "," << t1 << "," << t2 << "," << compensation << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType move_speed_j(ResponseCollector& response_collector, JointConstRef joint, double t1, double t2, double gain,
                          double alpha, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_speed_j(" << Type::joint_to_string(joint) << "," << t1 << "," << t2 << "," << gain << "," << alpha
       << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType move_speed_l(ResponseCollector& response_collector, PointConstRef point, double t1, double t2, double gain,
                          double alpha, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "move_speed_l(" << Type::point_to_string(point) << "," << t1 << "," << t2 << "," << gain << "," << alpha
       << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType set_tool_out(ResponseCollector& response_collector, int voltage,
                                             int signal_0, int signal_1, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "tool_out " << (int)voltage << "," << (int)signal_0 << "," << (int)signal_1;
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }


  ReturnType gripper_rts_rhp12rn_select_mode(ResponseCollector& response_collector, GripperConnectionPoint conn_point,
                                             bool force, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "gripper_macro " << (int)GripperModel::Robotis_RH_P12_RN << "," << (int)conn_point << "," << (force ? 3 : 0)
       << ",0,0,0,0,0,0,0";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType gripper_rts_rhp12rn_set_force_limit(ResponseCollector& response_collector,
                                                 GripperConnectionPoint conn_point, int limit, double timeout = -1.,
                                                 bool return_on_error = false) {
    std::stringstream ss;
    ss << "gripper_macro " << (int)GripperModel::Robotis_RH_P12_RN << "," << (int)conn_point << ",1"
       << ",0," << limit << ",0,0,0,0,0";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType gripper_rts_rhp12rn_force_control(ResponseCollector& response_collector, GripperConnectionPoint conn_point,
                                               int target_force_ratio, double timeout = -1.,
                                               bool return_on_error = false) {
    std::stringstream ss;
    ss << "gripper_macro " << (int)GripperModel::Robotis_RH_P12_RN << "," << (int)conn_point << ",4"
       << ",0," << target_force_ratio << ",0,0,0,0,0";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType gripper_rts_rhp12rn_position_control(ResponseCollector& response_collector,
                                                  GripperConnectionPoint conn_point, int target_position_ratio,
                                                  double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "gripper_macro " << (int)GripperModel::Robotis_RH_P12_RN << "," << (int)conn_point << ",2"
       << ",0," << target_position_ratio << ",0,0,0,0,0";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType gripper_koras_tooling_core_initialization(ResponseCollector& response_collector,
                                                  GripperConnectionPoint conn_point, int target_torque, int target_speed,
                                                  double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "gripper_macro " << (int)GripperModel::KORAS_Tooling << "," << (int)conn_point << ",0"
       << ",0," << (int)target_torque << "," << (int)target_speed << ",0,0,0,0";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType gripper_koras_tooling_vaccum_control(ResponseCollector& response_collector,
                                                  GripperConnectionPoint conn_point, int vaccum_on_off,
                                                  double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "gripper_macro " << (int)GripperModel::KORAS_Tooling << "," << (int)conn_point << ",1"
       << ",0," << (int)vaccum_on_off << ",0,0,0,0,0";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType gripper_koras_tooling_finger_initialization(ResponseCollector& response_collector,
                                                  GripperConnectionPoint conn_point,
                                                  double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "gripper_macro " << (int)GripperModel::KORAS_Tooling << "," << (int)conn_point << ",2"
       << ",0,0,0,0,0,0,0";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType gripper_koras_tooling_finger_open_close(ResponseCollector& response_collector,
                                                  GripperConnectionPoint conn_point, int finger_open_close,
                                                  double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "gripper_macro " << (int)GripperModel::KORAS_Tooling << "," << (int)conn_point << ",3"
       << ",0," << (int)finger_open_close << ",0,0,0,0,0";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType gripper_koras_tooling_finger_goto(ResponseCollector& response_collector,
                                                  GripperConnectionPoint conn_point, int target_position,
                                                  double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "gripper_macro " << (int)GripperModel::KORAS_Tooling << "," << (int)conn_point << ",4"
       << ",0," << (int)target_position << ",0,0,0,0,0";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType task_load(ResponseCollector& response_collector, std::string program_name, double timeout = -1.,
                       bool return_on_error = true) {
    trim(program_name);
    if (program_name.empty()) {
      return ReturnType(ReturnType::Error, timeout);
    }

    std::stringstream ss;
    ss << "task load " << program_name;
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType task_play(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "task play";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType task_stop(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "task stop";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType task_pause(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false) {
    std::stringstream ss;
    ss << "task pause";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType task_resume(ResponseCollector& response_collector, bool collision, double timeout = -1.,
                         bool return_on_error = false) {
    std::stringstream ss;
    ss << "task resume_" << (collision ? "b" : "a");
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType rt_script_onoff(ResponseCollector& response_collector, bool on, double timeout = -1.,
                             bool return_on_error = false) {
    std::stringstream ss;
    ss << "rt_script_onoff(" << (on ? 1 : 0) << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  ReturnType rt_script(ResponseCollector& response_collector, const std::string& single_command, double timeout = -1.,
                       bool return_on_error = false) {
    std::stringstream ss;
    ss << "rt_script(" << single_command << ")";
    sock_.send(ss.str());
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  /// Even if 'return_on_error' is true, when receiving the error message (load nofile), this function returns error
  ReturnType wait_for_task_loaded(ResponseCollector& response_collector, double timeout = -1.,
                                  bool return_on_error = true) {
    const auto check_done = [=](const Response& res) {
      return res.type() == Response::Type::Info && res.category() == "load" && res.msg() == "done";
    };
    const auto check_no_file = [=](const Response& res) {
      return res.type() == Response::Type::Error && res.category() == "load" && res.msg() == "nofile";
    };

    const auto& check = [=](const Response& res) {
      return check_done(res) || check_no_file(res);
    };
    auto res = wait_until(response_collector, check, timeout, return_on_error);
    if (res.is_success() && check_no_file(response_collector.back())) {
      return {ReturnType::Error, res.remain_time()};
    }
    return res;
  }

  ReturnType wait_for_task_started(ResponseCollector& response_collector, double timeout = -1.,
                                   bool return_on_error = true) {
    const auto& check = [=](const Response& res) {
      return res.type() == Response::Type::Info && res.category() == "program" && res.msg() == "start";
    };
    return wait_until(response_collector, check, timeout, return_on_error);
  }

  ReturnType wait_for_task_finished(ResponseCollector& response_collector, double timeout = -1.,
                                    bool return_on_error = true) {
    const auto& check = [=](const Response& res) {
      return res.type() == Response::Type::Info && res.category() == "program" && res.msg() == "end";
    };
    return wait_until(response_collector, check, timeout, return_on_error);
  }

  /**
  * @brief A function that evaluates a script on the 'Cobot'.
  *
  * This function sends the given script to the 'Cobot' for evaluation and waits for the response
  * from the 'Cobot'. In the case of failure, the function returns 'Timeout' or 'Error'.
  *
  * @param response_collector 'response_collector' stores the messages from 'Cobot' until this function returns.
  * @param script The script to be evaluated.
  * @param timeout The time in seconds that the function should wait for the response from 'Cobot'.
  * @param return_on_error If true, this function returns 'Error' when error comes from 'Cobot'.
  * @return ReturnType
  */
  ReturnType eval(ResponseCollector& response_collector, const std::string& script, double timeout = -1.,
                  bool return_on_error = false) {
    if (!sock_.send(script)) {
      throw std::runtime_error("send message failed");
    }
    return wait_until_ack_message(response_collector, timeout, return_on_error);
  }

  bool disable_waiting_ack(ResponseCollector& response_collector) {
    (void)response_collector;
    waiting_ack = false;
    return true;
  }

  bool enable_waiting_ack(ResponseCollector& response_collector) {
    flush(response_collector);
    response_collector.ack();

    eval(response_collector, R"(print("enable_waiting_ack","",""))");
    std::string val;
    auto res = wait_for_printed_value(response_collector, val, std::regex("enable_waiting_ack"));

    waiting_ack = true;
    response_collector.ack();
    if (res.is_success()) {
      return true;
    }
    return false;
  }

  void flush(ResponseCollector& response_collector) {
    while (_read_response_collector_from_buffer(response_collector))
      ;
  }

  template <typename T>
  ReturnType wait_for_printed_value(ResponseCollector& response_collector, T& out, const std::regex& pattern,
                                    double timeout = -1., bool return_on_error = true) {
    const auto& print_tag = std::regex("print_.*");
    const auto& check = [=](const Response& res) {
      return res.type() == Response::Type::Info && std::regex_match(res.category(), print_tag) &&
             std::regex_match(res.msg(), pattern);
    };
    auto ret = wait_until(response_collector, check, timeout, return_on_error);
    if (ret.is_success()) {
      std::string data = std::regex_replace(response_collector.back().msg(), pattern, "$1");
      if constexpr (std::is_same_v<T, JointRef> || std::is_same_v<T, PointRef> || std::is_same_v<T, Joint> ||
                    std::is_same_v<T, Point>) {
        std::stringstream ss(data);
        for (int i = 0; i < 6; i++) {
          double num;
          char comma;
          ss >> num >> comma;
          out[i] = num;
        }
      } else if constexpr (std::is_same_v<T, std::string>) {
        out = data;
      } else if constexpr (std::is_arithmetic_v<T> && !std::is_const_v<T>) {
        std::stringstream ss(data);
        ss >> out;
      }
    }
    return ret;
  }

  ReturnType wait_until(ResponseCollector& response_collector, const std::function<bool(const Response&)>& func,
                        double timeout = -1., bool return_on_error = false) {
    auto to = Timeout(timeout);
    auto already_checked = ResponseCollector();
    std::optional<ReturnType> ret;

    if ((response_collector.flag() & ResponseCollector::EnableCheckOldResponses) !=
        ResponseCollector::EnableCheckOldResponses) {
      already_checked.swap_responses(response_collector);
      // std::swap(already_checked, response_collector);
    }

    while (is_alive()) {
      while (!response_collector.empty()) {
        auto res = response_collector.front();
        response_collector.pop_front();
        already_checked.push_back(res);

        if (func(res)) {
          ret = ReturnType(ReturnType::Success, to.remain_time());
          break;
        }
        if (return_on_error && res.type() == Response::Type::Error) {
          ret = ReturnType(ReturnType::Error, to.remain_time());
          break;
        }
      }
      if (ret.has_value()) {
        break;
      }

      //      if (to.is_elapsed()) {
      //        ret = ReturnType(ReturnType::Timeout, to.remain_time());
      //        break;
      //      }

      while (!_read_response_collector_from_buffer(response_collector)) {
        if (to.is_elapsed()) {
          ret = ReturnType(ReturnType::Timeout, to.remain_time());
          break;
        }
        if (!is_alive()) {
          ret = ReturnType(ReturnType::Error, to.remain_time());
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
      if (ret.has_value()) {
        break;
      }
    }
    while (!response_collector.empty()) {
      already_checked.emplace_back(std::move(response_collector.front()));
      response_collector.pop_front();
    }
    already_checked.swap_responses(response_collector);
    // std::swap(already_checked, response_collector);
    return ret.value();
  }

  ReturnType wait_until_ack_message(ResponseCollector& response_collector, double timeout = -1.,
                                    bool return_on_error = false) {
    if (!waiting_ack) {
      return ReturnType(ReturnType::Success, timeout);
    }

    const auto& check = [=](const Response& res) {
      return res.type() == Response::Type::ACK;
    };
    auto res = wait_until(response_collector, check, timeout, return_on_error);
    if (res.is_success() && (response_collector.flag() & ResponseCollector::RemoveAckAutomatically) ==
                                ResponseCollector::RemoveAckAutomatically) {
      response_collector.pop_back();
    }
    return res;
  }

  ReturnType wait_for_move_started(ResponseCollector& response_collector, double timeout = -1.,
                                   bool return_on_error = true) {
    const auto& check = [=](const Response& res) {
      return res.type() == Response::Type::Info && res.category() == "motion_changed" && res.msg() != "0";
    };
    return wait_until(response_collector, check, timeout, return_on_error);
  }

  ReturnType wait_for_move_finished(ResponseCollector& response_collector, double timeout = -1.,
                                    bool return_on_error = true) {
    const auto& check = [=](const Response& res) {
      return res.type() == Response::Type::Info && res.category() == "motion_changed" && res.msg() == "0";
    };
    return wait_until(response_collector, check, timeout, return_on_error);
  }

 protected:
  bool _read_response_collector_from_buffer(ResponseCollector& response_collector) {
    auto recv_msg = sock_.recv();
    if (recv_msg.has_value()) {
      std::string msg = recv_msg.value();
      trim(msg, true);

      std::string m;
      std::stringstream ss(msg);
      while (getline(ss, m, '\n')) {
        trim(m, true);
        if (!m.empty()) {
          msgs_.emplace(m);
        }
      }
    }
    if (msgs_.empty()) {
      return false;
    } else {
      response_collector.add(Response(std::move(msgs_.front())));
      msgs_.pop();
    }
    return true;
  }

  [[nodiscard]] virtual bool is_alive() const { return true; }

 private:
  std::string address_;
  int port_;

  Socket sock_;

  std::queue<std::string> msgs_{};
  bool waiting_ack{true};
};

inline std::string to_string(Response::Type type) {
  switch (type) {
    case Response::Type::ACK:
      return "ACK";
    case Response::Type::Info:
      return "Info";
    case Response::Type::Warn:
      return "Warn";
    case Response::Type::Error:
      return "Error";
    case Response::Type::Unknown:
      return "Unknown";
  }
  return "";
}

inline std::string to_string(ReturnType::Type type) {
  switch (type) {
    case ReturnType::Undefined:
      return "Undefined";
    case ReturnType::Success:
      return "Success";
    case ReturnType::Timeout:
      return "Timeout";
    case ReturnType::Error:
      return "Error";
  }
  return "";
}

}  // namespace rb::podo
