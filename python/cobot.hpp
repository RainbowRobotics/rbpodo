/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#pragma once

#include <Eigen/Core>

#include "common.hpp"
#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/numpy.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "rbpodo/cobot.hpp"

namespace py = pybind11;
using namespace rb::podo;

template <typename T>
class PyCobot : public Cobot<EigenVector> {
 public:
  PyCobot(const std::string& cb_address, int command_port) : Cobot(cb_address, command_port) {}

  ASYNC_FUNC_WRAPPER_RC(Cobot, activate, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, shutdown, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_operation_mode, (OperationMode, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_speed_bar, (double, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_payload_info, (double, _a)(double, _b)(double, _c)(double, _d)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_tool_box, (double, _a)(double, _b)(double, _c)(double, _d)(double, _e)(double, _f)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_tcp_info, (PointConstRef, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_user_coordinate, (int, _a)(PointConstRef, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_freedrive_mode, (bool, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_collision_onoff, (bool, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_collision_threshold, (double, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_collision_mode, (CollisionMode, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_collision_after, (CollisionReactionMode, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_speed_multiplier, (double, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_acc_multiplier, (double, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_speed_acc_j, (double, _a)(double, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_speed_acc_l, (double, _a)(double, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_box_dout, (int, _a)(DigitalIOMode, mode)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_box_aout, (int, _a)(double, voltage)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_box_dout_toggle, (int, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_dout_bit_combination,
                        (int, _a)(int, _b)(unsigned int, _c)(Endian, _d)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_serial_tool, (int, _a)(int, _b)(int, _c)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, set_serial_box, (int, _a)(int, _b)(int, _c)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_l, (PointConstRef, _a)(double, _b)(double, _c)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_l_rel,
                        (PointConstRef, _a)(double, _b)(double, _c)(ReferenceFrame, _d)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_j, (JointConstRef, _a)(double, _b)(double, _c)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_jl, (PointConstRef, _a)(double, _b)(double, _c)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_pb_clear, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_pb_add,
                        (PointConstRef, _a)(double, _b)(BlendingOption, _c)(double, _d)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_pb_run, (double, _a)(MovePBOption, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_itpl_clear, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_itpl_add, (PointConstRef, _a)(double, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_itpl_run, (double, _a)(MoveITPLOption, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_lc_clear, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_lc_add,
                        (PointConstRef, _a)(double, _b)(MoveLCProperty, _c)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_lc_run, (double, _a)(MoveLCOption, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_lb_clear, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_lb_add, (PointConstRef, _a)(double, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_lb_run, (double, _a)(double, _b)(MoveLBOption, _c)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_c_points,
                        (PointConstRef, _a)(PointConstRef, _b)(double, _c)(double, _d)(MoveCOrientationOption,
                                                                                       _e)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_c_axis,
                        (PointConstRef, _a)(double, _b)(double, _c)(double, _d)(double, _e)(double, _f)(double, _g)(
                            MoveCRotationOption, _h)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_jb_clear, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_jb_add, (JointConstRef, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_jb_run, (double, _a)(double, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_jb2_clear, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_jb2_add,
                        (JointConstRef, _a)(double, _b)(double, _c)(double, _d)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_jb2_run, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_servo_j,
                        (JointConstRef, _a)(double, _b)(double, _c)(double, _d)(double, _e)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_servo_l,
                        (PointConstRef, _a)(double, _b)(double, _c)(double, _d)(double, _e)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_servo_t,
                        (PointConstRef, _a)(double, _b)(double, _c)(int, _d)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_speed_j,
                        (JointConstRef, _a)(double, _b)(double, _c)(double, _d)(double, _e)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, move_speed_l,
                        (PointConstRef, _a)(double, _b)(double, _c)(double, _d)(double, _e)(double, _to)(bool, _roe))

  ASYNC_FUNC_WRAPPER_RC(Cobot, set_tool_out,
                        (int, _a)(int, _b)(int, _c)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_rts_rhp12rn_select_mode,
                        (GripperConnectionPoint, _a)(int, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_rts_rhp12rn_set_force_limit,
                        (GripperConnectionPoint, _a)(int, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_rts_rhp12rn_force_control,
                        (GripperConnectionPoint, _a)(int, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_rts_rhp12rn_position_control,
                        (GripperConnectionPoint, _a)(int, _b)(double, _to)(bool, _roe))

  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_koras_tooling_core_initialization,
                        (GripperConnectionPoint, _a)(int, _b)(int, _c)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_koras_tooling_vaccum_control,
                        (GripperConnectionPoint, _a)(int, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_koras_tooling_finger_initialization,
                        (GripperConnectionPoint, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_koras_tooling_finger_open_close,
                        (GripperConnectionPoint, _a)(int, _b)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_koras_tooling_finger_goto,
                        (GripperConnectionPoint, _a)(int, _b)(double, _to)(bool, _roe))
                        
  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_inspire_humanoid_hand_initialization,
                        (int, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, gripper_inspire_humanoid_hand_set_finger,
                        (int, _a)(int, _b)(int, _c)(int, _d)(int, _e)(int, _f)(int, _g)(double, _to)(bool, _roe))

  ASYNC_FUNC_WRAPPER_RC(Cobot, task_load, (std::string, _a)(double, _to)(bool, _roe))  // NOLINT
  ASYNC_FUNC_WRAPPER_RC(Cobot, task_play, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, task_stop, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, task_pause, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, task_resume, (bool, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, rt_script_onoff, (bool, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, rt_script, (const std::string&, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, wait_for_task_loaded, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, wait_for_task_started, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, wait_for_task_finished, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, eval, (const std::string&, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, disable_waiting_ack)
  ASYNC_FUNC_WRAPPER_RC(Cobot, enable_waiting_ack)
  ASYNC_FUNC_WRAPPER_RC(Cobot, flush)
  ASYNC_FUNC_WRAPPER_RC(Cobot, wait_until, (const std::function<bool(const Response&)>&, _a)(double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, wait_until_ack_message, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, wait_for_move_started, (double, _to)(bool, _roe))
  ASYNC_FUNC_WRAPPER_RC(Cobot, wait_for_move_finished, (double, _to)(bool, _roe))

  [[nodiscard]] bool is_alive() const override {
    py::gil_scoped_acquire g{};
    bool interrupt = (PyErr_CheckSignals() == 0);
    if (!interrupt) {
      throw py::error_already_set();
    }
    return interrupt;
  }
};

void pybind11_error_code(py::module_& m) {
  py::class_<Message>(m, "Message")
      .def_readonly("ko", &Message::ko)
      .def_readonly("en", &Message::en)
      .def("__repr__", [](const Message& res) {
        std::stringstream ss;
        ss << R"({ "English": ")" << res.en << R"(", "Korean": ")" << res.ko << "\" }";
        return ss.str();
      });
  m.attr("ErrorCodeMessage") = ErrorCodeMessage;
}

void pybind11_response(py::module_& m) {
  py::class_<Response> response(m, "Response");
  py::enum_<Response::Type>(response, "Type")
      .value("ACK", Response::Type::ACK)
      .value("Info", Response::Type::Info)
      .value("Warn", Response::Type::Warn)
      .value("Error", Response::Type::Error)
      .value("Unknown", Response::Type::Unknown);
  response.def("raw", &Response::raw, py::return_value_policy::reference)
      .def("type", &Response::type)
      .def("category", &Response::category, py::return_value_policy::reference)
      .def("msg", &Response::msg, py::return_value_policy::reference)
      .def("__repr__", [](const Response& res) { return res.str(); });

  py::class_<ResponseCollector> response_collector(m, "ResponseCollector");
  py::enum_<ResponseCollector::ConfigFlag>(response_collector, "ConfigFlag")
      .value("Default", ResponseCollector::Default)
      .value("EnableCheckOldResponses", ResponseCollector::EnableCheckOldResponses)
      .value("RemoveAckAutomatically", ResponseCollector::RemoveAckAutomatically);
  response_collector
      .def(py::init<unsigned int>(), py::arg("flag") = (unsigned int)ResponseCollector::RemoveAckAutomatically)
      .def("type_filter", &ResponseCollector::type_filter, py::arg("type"), py::arg("remain") = false)
      .def("add", pybind11::overload_cast<const Response&>(&ResponseCollector::add), py::arg("response"))
      .def("set_callback", &ResponseCollector::set_callback, py::arg("callback"))
      .def("clear_callback", &ResponseCollector::clear_callback)
      .def("clear", &ResponseCollector::clear)
      .def("info", &ResponseCollector::info, py::arg("remain") = false)
      .def("warn", &ResponseCollector::warn, py::arg("remain") = false)
      .def("error", &ResponseCollector::error, py::arg("remain") = false)
      .def("ack", &ResponseCollector::ack, py::arg("remain") = false)
      .def("category_filter", &ResponseCollector::category_filter, py::arg("filter"), py::arg("remain") = false)
      .def("has_error", &ResponseCollector::has_error)
      .def("throw_if_not_empty", &ResponseCollector::throw_if_not_empty)
      .def("flag", &ResponseCollector::flag)
      .def("set_flag", &ResponseCollector::set_flag)
      .def("__len__", [](const ResponseCollector& self) { return self.size(); })
      .def(
          "__iter__", [](const ResponseCollector& self) { return py::make_iterator(self.begin(), self.end()); },
          py::keep_alive<0, 1>())
      .def("__repr__", [](const ResponseCollector& self) {
        std::stringstream ss;
        ss << self;
        return ss.str();
      });

  py::class_<ReturnType> rt(m, "ReturnType");
  py::enum_<ReturnType::Type>(rt, "Type")
      .value("Undefined", ReturnType::Undefined)
      .value("Success", ReturnType::Success)
      .value("Timeout", ReturnType::Timeout)
      .value("Error", ReturnType::Error)
      .export_values();
  rt.def(py::init<ReturnType::Type, double>(), py::arg("type"), py::arg("remain_time"))
      .def("type", &ReturnType::type)
      .def("remain_time", &ReturnType::remain_time)
      .def("set_remain_time", &ReturnType::set_remain_time, py::arg("remain_time"))
      .def("is_success", &ReturnType::is_success)
      .def("is_timeout", &ReturnType::is_timeout)
      .def("is_error", &ReturnType::is_error)
      .def("__repr__", [](const ReturnType& self) {
        std::stringstream ss;
        ss << self;
        return ss.str();
      });
}

template <typename T>
void pybind11_cobot(py::module_& m) {
  py::class_<PyCobot<T>> cobot(m, "Cobot");
  cobot
      .def(py::init<const std::string&, int>(), py::arg("address"), py::arg("port") = kCommandPort, R"pbdoc(
Class is for interacting with Rainbow Robotics Cobot.

Parameters
----------
address : str
    IP address for command channel (e.g. 10.0.2.7). You can set up via teaching pendant (UI)
port : int
    a port number for command channel (default: 5000).

Example
-------
>>> robot = rb.cobot("10.0.2.7")
>>> rc = rb.ResponseCollector()
>>> robot.set_operation_mode(rc, rb.OperationMode.Real)
)pbdoc")
      .def(
          "get_control_box_info",
          [](PyCobot<T>* self, ResponseCollector* response_collector, double timeout, bool return_on_error) {
            auto _f = [=] {
              ControlBoxInfo cbi;
              auto res = static_cast<Cobot<EigenVector>&>(*self).get_control_box_info(*response_collector, cbi, timeout,
                                                                                      return_on_error);
              return std::make_pair(res, cbi);
            };
            ASYNC_FUNC_WRAPPER_HELPER(_f)
          },
          py::arg("response_collector"), py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
Retrieves information about the control box.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

ControlBoxInfo
    Information including "system version" and "robot box type"

Examples
--------
>>> res, info = robot.get_control_box_info(rc)
>>> print(info)
{ "SystemVersion": 24021504, "RobotBoxType": 11 }
)pbdoc")
      .def(
          "get_robot_state",
          [](PyCobot<T>* self, ResponseCollector* response_collector, double timeout, bool return_on_error) {
            auto _f = [=] {
              RobotState rs;
              auto res = static_cast<Cobot<EigenVector>&>(*self).get_robot_state(*response_collector, rs, timeout,
                                                                                 return_on_error);
              return std::make_pair(res, rs);
            };
            ASYNC_FUNC_WRAPPER_HELPER(_f)
          },
          py::arg("response_collector"), py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("activate", &PyCobot<T>::activate, py::arg("response_collector"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = true,
           R"pbdoc(
Turn on the power supply for the robot arm. If the robot is already activated or has some errors, it returns immediately.

Warning
-------
The robot arm will power up. Be careful when you use this function.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("shutdown", &PyCobot<T>::shutdown, py::arg("response_collector"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = true,
           R"pbdoc(
Turn off the power supply for the robot arm.

Warning
-------
The robot arm powers up. Be careful when you use this function.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def(
          "get_system_variable",
          [](PyCobot<T>* self, rb::podo::ResponseCollector* rc, rb::podo::SystemVariable system_variable,
             double timeout, bool return_on_error) {
            auto _f = [=] {
              double out;
              auto res = static_cast<Cobot<EigenVector>&>(*self).get_system_variable(*rc, system_variable, out, timeout,
                                                                                     return_on_error);
              return std::make_pair(res, out);
            };
            ASYNC_FUNC_WRAPPER_HELPER(_f)
          },
          py::arg("response_collector"), py::arg("system_variable"), py::arg("timeout") = -1.,
          py::arg("return_on_error") = false)
      .def(
          "print_variable",
          [](PyCobot<T>* self, ResponseCollector* response_collector, const std::string& variable_name, double timeout,
             bool return_on_error) {
            auto _f = [=] {
              std::string str;
              auto res = static_cast<Cobot<EigenVector>&>(*self).print_variable(*response_collector, variable_name, str,
                                                                                timeout, return_on_error);
              return std::make_pair(res, str);
            };
            ASYNC_FUNC_WRAPPER_HELPER(_f)
          },
          py::arg("response_collector"), py::arg("variable_name"), py::arg("timeout") = -1.,
          py::arg("return_on_error") = false)
      .def("set_operation_mode", &PyCobot<T>::set_operation_mode, py::arg("response_collector"), py::arg("mode"),
           py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Change the operation mode between real and simulation modes.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
mode : rbpodo.OperationMode
    If set to OperationMode.Real, the robot moves when commanded.
    If set to OperationMode.Simulation, the robot does not moves but the internal reference values changes.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def(
          "get_tcp_info",
          [](PyCobot<T>* self, ResponseCollector* response_collector, double timeout, bool return_on_error) {
            auto _f = [=] {
              typename EigenVector::Point pnt;
              auto res = static_cast<Cobot<EigenVector>&>(*self).get_tcp_info(*response_collector, pnt, timeout,
                                                                              return_on_error);
              return std::make_pair(res, pnt);
            };
            ASYNC_FUNC_WRAPPER_HELPER(_f)
          },
          py::arg("response_collector"), py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
This function returns the TCP information of the current robot.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

numpy.ndarray(shape=(6, 1))
    TCP of the current robot with respect to the global coordinate system. (Unit: mm & degree)


Examples
--------
>>> [res, pnt] = robot.get_tcp_info(rc)
)pbdoc")
      .def(
          "get_tfc_info",
          [](PyCobot<T>* self, ResponseCollector* response_collector, double timeout, bool return_on_error) {
            auto _f = [=] {
              typename EigenVector::Point pnt;
              auto res = static_cast<Cobot<EigenVector>&>(*self).get_tfc_info(*response_collector, pnt, timeout,
                                                                              return_on_error);
              return std::make_pair(res, pnt);
            };
            ASYNC_FUNC_WRAPPER_HELPER(_f)
          },
          py::arg("response_collector"), py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
This function returns the TFC (Tool flange center) information of the current robot.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

numpy.ndarray(shape=(6, 1))
    TFC of the current robot based on the global coordinate system. (Unit: mm & degree)


Examples
--------
>>> [res, pnt] = robot.get_tfc_info(rc)
)pbdoc")
      .def("set_speed_bar", &PyCobot<T>::set_speed_bar, py::arg("response_collector"), py::arg("speed"),
           py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Set the overall speed control bar. (bottom speed control bar in UI).

Warning
-------
When running a program on the UI Make page, this function does not work if the safety slide bar option is turned on.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
speed : float
    Desired speed control bar position (0~1)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_payload_info", &PyCobot<T>::set_payload_info, py::arg("response_collector"), py::arg("weight"),
           py::arg("com_x"), py::arg("com_y"), py::arg("com_z"), py::arg("timeout") = -1,
           py::arg("return_on_error") = false, R"pbdoc(
Set the tool payload w.r.t. the manufacturer’s default tool coordinate system.

Warning
-------
The value set in this function returns to the default value after the program ends.
If this function is not called in program-flow, the value set in the Setup page is used.
During program flow, the value set in this function is maintained until this function is called again.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
weight : float
    payload weight (Unit: Kg)
com_x : float
    payload center of mass x-axis value with respect to the manufacturer's default coordinate system. (Unit: mm)
com_y : float
    payload center of mass y-axis value with respect to the manufacturer's default coordinate system. (Unit: mm)
com_z : float
    payload center of mass z-axis value with respect to the manufacturer's default coordinate system. (Unit: mm)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_tool_box", &PyCobot<T>::set_tool_box, py::arg("response_collector"), py::arg("x_width"),
           py::arg("y_width"), py::arg("z_width"), py::arg("x_offset"), py::arg("y_offset"), py::arg("z_offset"), py::arg("timeout") = -1,
           py::arg("return_on_error") = false, R"pbdoc(
Set the tool box w.r.t. the manufacturer’s default tool coordinate system.

Warning
-------
The value set in this function returns to the default value after the program ends.
If this function is not called in program-flow, the value set in the Setup page is used.
During program flow, the value set in this function is maintained until this function is called again.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
x_width : float
    width of tool along x-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
y_width : float
    width of tool along y-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
z_width : float
    width of tool along z-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
x_offset : float
    offset of box along x-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
y_offset : float
    offset of box along y-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
z_offset : float
    offset of box along z-axis with respect to the manufacturer's default tool coordinate system. (Unit: mm)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_tcp_info", &PyCobot<T>::set_tcp_info, py::arg("response_collector"), py::arg("point"),
           py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Set the TCP position and orientation w.r.t. the manufacturer’s default tool coordinate system.

Warning
-------
The value set in this function returns to the default value after the program ends.
If this function is not called in program-flow, the value set in the Setup page is used.
During program flow, the value set in this function is maintained until this function is called again.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
point : numpy.ndarray(shape=(6, 1))
    position and orientation of tcp with respect to manufacturer's default tool coordinate system. (x, y, z, rx, ry, rz) (Unit: mm & degree)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_user_coordinate", &PyCobot<T>::set_user_coordinate, py::arg("response_collector"), py::arg("id"),
           py::arg("point"), py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Set user coordinate

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
id : int
    id of user coordinate to change (0~2)
point : numpy.ndarray(shape=(6, 1))
    position and orientation of coordinate with respect to base frame. (x, y, z, rx, ry, rz) (Unit: mm & degree)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_freedrive_mode", &PyCobot<T>::set_freedrive_mode, py::arg("response_collector"), py::arg("on"),
           py::arg("timeout") = -1, py::arg("return_on_error") = false)
      .def("set_collision_onoff", &PyCobot<T>::set_collision_onoff, py::arg("response_collector"), py::arg("on"),
           py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
This function turns on/off the collision detection function.

Warning
-------
The value set in this function returns to the default value after the program ends.
If this function is not called in program-flow, the value set in the Setup page is used.
During program flow, the value set in this function is maintained until this function is called again.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
on : bool
    The variable represents an on/off state, where 0 is off and 1 is on.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_collision_threshold", &PyCobot<T>::set_collision_threshold, py::arg("response_collector"),
           py::arg("threshold"), py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Sets the collision sensitivity (threshold).

Warning
-------
The value set in this function returns to the default value after the program ends.
If this function is not called in program-flow, the value set in the Setup page is used.
During program flow, the value set in this function is maintained until this function is called again.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
threshold : float
    The variable represents an threshold value. The lower the value, the more sensitive to collision. (0~1)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_collision_mode", &PyCobot<T>::set_collision_mode, py::arg("response_collector"), py::arg("mode"),
           py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Set the stop-mode after the collision detection.

Warning
-------
The value set in this function returns to the default value after the program ends.
If this function is not called in program-flow, the value set in the Setup page is used.
During program flow, the value set in this function is maintained until this function is called again.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
mode : CollisionMode
    The variable represents a stop mode.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_collision_after", &PyCobot<T>::set_collision_after, py::arg("response_collector"), py::arg("mode"),
           py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Set the program flow direction after the collision detection.

Warning
-------
The value set in this function returns to the default value after the program ends.
If this function is not called in program-flow, the value set in the Setup page is used.
During program flow, the value set in this function is maintained until this function is called again.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
mode : CollisionReactionMode
    A variable represents a stop mode.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_speed_multiplier", &PyCobot<T>::set_speed_multiplier, py::arg("response_collector"),
           py::arg("multiplier"), py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Sets the overall speed (velocity) multiplier.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
multiplier : float
    Multiply variable. (0~2) Default value is 1.0.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_acc_multiplier", &PyCobot<T>::set_acc_multiplier, py::arg("response_collector"), py::arg("multiplier"),
           py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Sets the overall acceleration multiplier.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
multiplier : float
    Multiply variable. (0~2) Default value is 1.0.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_speed_acc_j", &PyCobot<T>::set_speed_acc_j, py::arg("response_collector"), py::arg("speed"),
           py::arg("acceleration"), py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Sets fixed joint velocity/acceleration for J-series motions (MoveJ, MoveJB, MoveJL).

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
speed : float
    Speed/velocity (Unit: deg/s). Does not use negative value.
acceleration : float
    Acceleration (Unit: deg/s^2). Does not use negative value.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_speed_acc_l", &PyCobot<T>::set_speed_acc_l, py::arg("response_collector"), py::arg("speed"),
           py::arg("acceleration"), py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Sets fixed linear velocity/acceleration for L-series motions (MoveL, MovePB, MoveLB, MoveITPL).

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
speed : float
    Speed/velocity (Unit: mm/s)
acceleration : float
    Acceleration (Unit: mm/s^2)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_box_dout", &PyCobot<T>::set_box_dout, py::arg("response_collector"), py::arg("port"), py::arg("mode"),
           py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Set the digital output of the control box.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
port : int
    Port number for the digital output.
mode : DigitalIOMode
    Output mode selection
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_box_aout", &PyCobot<T>::set_box_aout, py::arg("response_collector"), py::arg("port"),
           py::arg("voltage"), py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Set the analog output of the control box.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
port : int
    Port number for the analog output. (0~15)
voltage : float
    Desired output voltage (0~10V, Unit: V)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_box_dout_toggle", &PyCobot<T>::set_box_dout_toggle, py::arg("response_collector"), py::arg("port"),
           py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Toggles the current digital output of the control box.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
port : int
    Port number for the analog output. (0~15)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("set_dout_bit_combination", &PyCobot<T>::set_dout_bit_combination, py::arg("response_collector"),
           py::arg("first_port"), py::arg("last_port"), py::arg("value"), py::arg("mode"), py::arg("timeout") = -1,
           py::arg("return_on_error") = false, R"pbdoc(
Set the digital outputs of the control box simultaneously.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
first_port : int
    First port number (0~15)
last_port : int
    Last port number (0~15)
    last_port must be greater or equal than first_port.
value : int
    Output value for digital ports (bit combination)
    If mode is LittleEndian and value is 5, then port 0 and port 2 will be high.
mode : rbpodo.Endian
    Endian selection
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> robot.set_dout_bit_combination(rc, 0, 15, 5, rb.Endian.LittleEndian)  # Port 0 and 2 will be high, while port 1 and 3 are low
>>> robot.set_dout_bit_combination(rc, 0, 15, 10, rb.Endian.LittleEndian) # Port 1 and 3 will be high, while port 0 and 2 are low
)pbdoc")
      .def("set_serial_tool", &PyCobot<T>::set_serial_tool, py::arg("response_collector"), py::arg("baud_rate"),
           py::arg("stop_bit"), py::arg("parity_bit"), py::arg("timeout") = -1, py::arg("return_on_error") = false,
           R"pbdoc(
Set the serial communication (RS232/485) provided by the Tool Flange of the robot arm.

Warning
-------
The value set in this function returns to the default value after the program ends.
If this function is not called in program-flow, the value set in the Setup page is used.
During program flow, the value set in this function is maintained until this function is called again.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
baud_rate : int
    Communication speed (Baud rate)
stop_bit : int
    Stop bit, (0 or 1, Default value is 1)
parity_bit : int
    Parity bit, (0: none, 1: odd, 2: even, Default value is 0)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> robot.set_serial_tool(rc, 115200, 1, 0) # Set tool-flange serial comm. : baud rate = 115200 / stop bit = 1 / parity = none
)pbdoc")
      .def("set_serial_box", &PyCobot<T>::set_serial_box, py::arg("response_collector"), py::arg("baud_rate"),
           py::arg("stop_bit"), py::arg("parity_bit"), py::arg("timeout") = -1, py::arg("return_on_error") = false,
           R"pbdoc(
Set the serial communication (RS232/485) provided by the control box.

Warning
-------
The value set in this function returns to the default value after the program ends.
If this function is not called in program-flow, the value set in the Setup page is used.
During program flow, the value set in this function is maintained until this function is called again.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
baud_rate : int
    Communication speed (Baud rate)
stop_bit : int
    Stop bit, (0 or 1, Default value is 1)
parity_bit : int
    Parity bit, (0: none, 1: odd, 2: even, Default value is 0)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def(
          "calc_fk_tcp",
          [](PyCobot<T>* self, ResponseCollector* response_collector, py::array_t<double>* point, double j0, double j1,
             double j2, double j3, double j4, double j5, double timeout, bool return_on_error) {
            assert(point->size() == 6);
            auto _f = [=] {
              typename EigenVector::Point pnt;
              auto res = static_cast<Cobot<EigenVector>&>(*self).calc_fk_tcp(*response_collector, pnt, j0, j1, j2, j3,
                                                                             j4, j5, timeout, return_on_error);
              for (int i = 0; i < 6; i++) {
                *(point->mutable_data(i)) = pnt[i];
              }
              return res;
            };
            ASYNC_FUNC_WRAPPER_HELPER(_f)
          },
          py::arg("response_collector"), py::arg("point"), py::arg("j0"), py::arg("j1"), py::arg("j2"), py::arg("j3"),
          py::arg("j4"), py::arg("j5"), py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Calculate TCP posture w.r.t. global (base) coordinate from six joint angles.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
point : numpy.ndarray(shape=(6, 1))
    TCP pose [x, y, z, rx, ry, rz] w.r.t. global (base) coordinate
j0, j1, j2, j3, j4, j5: joint angles (unit: degree)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> pnt = np.zeros((6,))
>>> res = robot.calc_fk_tcp(rc, pnt, 0, 0, 0, 0, 0, 0)
)pbdoc")
      .def(
          "calc_fk_tcp",
          [](PyCobot<T>* self, ResponseCollector* response_collector, double j0, double j1, double j2, double j3,
             double j4, double j5, double timeout, bool return_on_error) {
            auto _f = [=] {
              typename EigenVector::Point pnt;
              auto res = static_cast<Cobot<EigenVector>&>(*self).calc_fk_tcp(*response_collector, pnt, j0, j1, j2, j3,
                                                                             j4, j5, timeout, return_on_error);
              return std::make_pair(res, pnt);
            };
            ASYNC_FUNC_WRAPPER_HELPER(_f)
          },
          py::arg("response_collector"), py::arg("j0"), py::arg("j1"), py::arg("j2"), py::arg("j3"), py::arg("j4"),
          py::arg("j5"), py::arg("timeout") = -1, py::arg("return_on_error") = false, R"pbdoc(
Calculate TCP posture w.r.t. global (base) coordinate from six joint angles.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
j0, j1, j2, j3, j4, j5: joint angles (unit: degree)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

numpy.ndarray(shape=(6, 1))
    TCP pose [x, y, z, rx, ry, rz] w.r.t. global (base) coordinate


Examples
--------
>>> [res, pnt] = robot.calc_fk_tcp(rc, 0, 0, 0, 0, 0, 0, -1, False)
)pbdoc")
      .def(
          "calc_fk_tcp",
          [](PyCobot<T>* self, ResponseCollector* response_collector, py::array_t<double>* point,
             typename EigenVector::JointConstRef joint, double timeout, bool return_on_error) {
            assert(point->size() == 6);
            auto _f = [=] {
              typename EigenVector::Point pnt;
              auto res = static_cast<Cobot<EigenVector>&>(*self).calc_fk_tcp(*response_collector, pnt, joint, timeout,
                                                                             return_on_error);
              for (int i = 0; i < 6; i++) {
                *(point->mutable_data(i)) = pnt[i];
              }
              return res;
            };
            ASYNC_FUNC_WRAPPER_HELPER(_f)
          },
          py::arg("response_collector"), py::arg("point"), py::arg("joint"), py::arg("timeout") = -1,
          py::arg("return_on_error") = false, R"pbdoc(
Calculate TCP posture w.r.t. global (base) coordinate from six joint angles.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
[out] point : numpy.ndarray(shape=(6, 1))
    TCP pose [x, y, z, rx, ry, rz] w.r.t. global (base) coordinate
joint : numpy.ndarray(shape=(6, 1))
    Single joint type variable which contains six joint-angles.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> pnt = np.zeros((6,))
>>> res = robot.calc_fk_tcp(rc, pnt, np.zeros((6,)))
)pbdoc")
      .def(
          "calc_fk_tcp",
          [](PyCobot<T>* self, ResponseCollector* response_collector, typename EigenVector::JointConstRef joint,
             double timeout, bool return_on_error) {
            auto _f = [=] {
              typename EigenVector::Point pnt;
              auto res = static_cast<Cobot<EigenVector>&>(*self).calc_fk_tcp(*response_collector, pnt, joint, timeout,
                                                                             return_on_error);
              return std::make_pair(res, pnt);
            };
            ASYNC_FUNC_WRAPPER_HELPER(_f)
          },
          py::arg("response_collector"), py::arg("joint"), py::arg("timeout") = -1., py::arg("return_on_error") = false,
          R"pbdoc(
Calculate TCP posture w.r.t. global (base) coordinate from six joint angles.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
joint : numpy.ndarray(shape=(6, 1))
    Single joint type variable which contains six joint-angles.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

point : numpy.ndarray(shape=(6, 1))
    TCP pose [x, y, z, rx, ry, rz] w.r.t. global (base) coordinate

Examples
--------
>>> [res, pnt] = robot.calc_fk_tcp(rc, np.zeros((6,)))
)pbdoc")
      .def("move_l", &PyCobot<T>::move_l, py::arg("response_collector"), py::arg("point"), py::arg("speed"),
           py::arg("acceleration"), py::arg("timeout") = -1., py::arg("return_on_err") = false, R"pbdoc(
A function that makes TCP to move in a straight line to the target point.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
point : numpy.ndarray(shape=(6, 1))
    Target TCP pose
speed : float
    Speed (unit: mm/s)
acceleration : float
    Acceleration (unit: mm/s^2)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> my_point1 = np.array([100, 200, 300, 0, 0, 0])
>>> my_point2 = np.array([100, 150, 100, 0, 90, 0])
>>> robot.move_l(rc, my_point1, 20, 5)
>>> robot.move_l(rc, my_point2, 20, 5)
)pbdoc")
      .def("move_l_rel", &PyCobot<T>::move_l_rel, py::arg("response_collector"), py::arg("point"), py::arg("speed"),
           py::arg("acceleration"), py::arg("frame"), py::arg("timeout") = -1, py::arg("return_on_error") = false,
           R"pbdoc(
A function that makes TCP to move in a straight line to the target point.
Enter the target point as a value relative to the current TCP value.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
point : numpy.ndarray(shape=(6, 1))
    Target TCP pose
speed : float
    Speed (unit: mm/s)
acceleration : float
    Acceleration (unit: mm/s^2)
frame : ReferenceFrame
    Reference frame for the relative point value.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> target_point = np.array([0, 100, -200, 0, 0, 0])
>>> robot.move_l_rel(rc, target_point, 300, 400, rb.ReferenceFrame.Base) # move TCP (0,100,-200) w.r.t. Base coordinate (speed/acceleration = 300 / 400)
)pbdoc")
      .def("move_j", &PyCobot<T>::move_j, py::arg("response_collector"), py::arg("joint"), py::arg("speed"),
           py::arg("acceleration"), py::arg("timeout") = -1., py::arg("return_on_err") = false, R"pbdoc(
Move the robot arm to the target joint angle in joint space.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
joint : numpy.ndarray(shape=(6, 1))
    Target joint angles. (Joint)
speed : float
    Speed (Unit: deg/s)
acceleration : float
    Acceleration (Unit: deg/s^2)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> robot.move_j(rc, {0, 0, 90, 0, 90, 0}, 60, 80) // move joint angles to (0,0,90,0,90,0) degree with speed/acceleration = 60/80.
)pbdoc")
      .def("move_jl", &PyCobot<T>::move_jl, py::arg("response_collector"), py::arg("point"), py::arg("speed"),
           py::arg("acceleration"), py::arg("timeout") = -1., py::arg("return_on_err") = false, R"pbdoc(
This function moves to the target point using the move_j method rather than a straight line.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
point : numpy.ndarray(shape=(6, 1))
    Target TCP posture. (Point)
speed : float
    Speed (Unit: deg/s)
acceleration : float
    Acceleration (Unit: deg/s^2)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> robot.move_jl(rc, {100, 200, 300, 0, 0, 0}, 20, 5) // Move TCP to '{x = 100, y = 200, z = 300, rx = 0, ry = 0, rz = 0}' via MoveJ method.
)pbdoc")
      .def("move_pb_clear", &PyCobot<T>::move_pb_clear, py::arg("response_collector"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false, R"pbdoc(
Initialize (Clear) the point list to be used in MovePB.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_pb_add", &PyCobot<T>::move_pb_add, py::arg("response_collector"), py::arg("point"), py::arg("speed"),
           py::arg("option"), py::arg("blending_value"), py::arg("timeout") = -1, py::arg("return_on_error") = false,
           R"pbdoc(
This function adds the points used in MovePB to the list.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
point : numpy.ndarray(shape=(6, 1))
    Target TCP posture. (Point)
speed : float
    Speed (Unit: mm/s)
option : BlendingOption
    Blending option (0: blend based on ratio, 1: blend based on distance.)
blending_value : float
    Blending value (0~1 in ratio option or distance in distance option (Unit: mm)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> robot.move_pb_clear(rc)
>>> robot.move_pb_add(rc, np.array([100, 200, 200, 90, 0, 0]), 200.0, rb.BlendingOption.Ratio, 0.5)
>>> robot.move_pb_add(rc, np.array([300, 300, 400, 0, 0, 0]), 400.0, rb.BlendingOption.Ratio, 0.5)
>>> robot.move_pb_add(rc, np.array([0, 200, 400, 90, 0, 0]), 200.0, rb.BlendingOption.Ratio, 0.5)
>>> robot.move_pb_run(rc, 800, rb.MovePBOption.Intended)
)pbdoc")
      .def("move_pb_run", &PyCobot<T>::move_pb_run, py::arg("response_collector"), py::arg("acceleration"),
           py::arg("option"), py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
This function executes MovePB using the points added in move_pb_add.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
acceleration : float
    Acceleration (Unit: mm/s^2)
option : MovePBOption
    Orientation option (0: Intended, 1: Constant)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_itpl_clear", &PyCobot<T>::move_itpl_clear, py::arg("response_collector"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false, R"pbdoc(
Initialize (Clear) the point list to be used in MoveITPL.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_itpl_add", &PyCobot<T>::move_itpl_add, py::arg("response_collector"), py::arg("point"),
           py::arg("speed"), py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
This function adds the points used in MoveITPL to the list.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
point : numpy.ndarray(shape=(6, 1))
    Target TCP posture. (Point)
speed : float
    Speed (Unit: mm/s)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_itpl_run", &PyCobot<T>::move_itpl_run, py::arg("response_collector"), py::arg("acceleration"),
           py::arg("option"), py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
This function executes MoveITPL using the points added in move_itpl_add.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
acceleration : float
    Acceleration
option : MoveITPLOption
    Orientation/motion option. (CA : Combined Arc mode)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_lc_clear", &PyCobot<T>::move_lc_clear, py::arg("response_collector"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false, R"pbdoc(
Initialize (Clear) the point list to be used in MoveLC.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_lc_add", &PyCobot<T>::move_lc_add, py::arg("response_collector"), py::arg("point"), py::arg("speed"),
           py::arg("property"), py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
This function adds the points used in MoveLC to the list.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
point : numpy.ndarray(shape=(6, 1))
    Target TCP posture. (Point)
speed : float
    Speed (Unit: mm/s)
property : MoveLCProperty
    0 or 1 (0 : Pass through linear motion, 1 : Pass through circular motion)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_lc_run", &PyCobot<T>::move_lc_run, py::arg("response_collector"), py::arg("acceleration"),
           py::arg("option"), py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
This function executes MoveITPL using the points added in move_itpl_add.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
acceleration : float
    Acceleration
option : MoveLCOption
    Orientation options
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_lb_clear", &PyCobot<T>::move_lb_clear, py::arg("response_collector"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false, R"pbdoc(
Initialize (Clear) the point list to be used in MoveLB.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_lb_add", &PyCobot<T>::move_lb_add, py::arg("response_collector"), py::arg("point"),
           py::arg("blend_distance"), py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
This function adds the points used in MoveLB to the list.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
point : numpy.ndarray(shape=(6, 1))
    Target TCP posture. (Point)
blend_distance : float
    Blend distance. (Unit: mm)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_lb_run", &PyCobot<T>::move_lb_run, py::arg("response_collector"), py::arg("speed"),
           py::arg("acceleration"), py::arg("option"), py::arg("timeout") = -1., py::arg("return_on_error") = false,
           R"pbdoc(
This function executes MoveLB using the points added in move_lb_add.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
speed : float
    Speed (Unit: mm/s)
acceleration : float
    Acceleration (Unit: mm/s^2)
option : MoveLBOption
    Orientation options. (0 : Intended, 1 : Constant)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_c_points", &PyCobot<T>::move_c_points, py::arg("response_collector"), py::arg("via_point"),
           py::arg("target_point"), py::arg("speed"), py::arg("acceleration"), py::arg("option"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
This function performs a movement that draws an arc through via & target points.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
via_point : numpy.ndarray(shape=(6, 1))
    via Point TCP posture.
target_point: numpy.ndarray(shape=(6, 1))
    target Point TCP posture.
speed : float
    Speed (Unit: mm/s)
acceleration : float
    Acceleration (Unit: mm/s^2)
option : MoveCOrientationOption
    Orientation options. (0 : Intended, 1 : Constant, 2 : Radial, 3 : Smooth)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_c_axis", &PyCobot<T>::move_c_axis, py::arg("response_collector"), py::arg("center"), py::arg("x_axis"),
           py::arg("y_axis"), py::arg("z_axis"), py::arg("angle"), py::arg("speed"), py::arg("acceleration"),
           py::arg("option"), py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
This function performs a movement that draws an arc through via & target points.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
center : numpy.ndarray(shape=(6, 1))
    Center of the rotation (Unit: mm)
x_axis : float
    Rotation axis's x axis vector
y_axis : float
    Rotation axis's y axis vector
z_axis : float
    Rotation axis's z axis vector
angle : float
    Rotation angle (Unit: deg)
speed : float
    Speed (Unit: mm/s)
acceleration : float
    Acceleration (Unit: mm/s^2)
option : MoveCRotationOption
    Rotation options. (0 : Intended, 1 : Constant, 2 : Radial)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> robot.move_c_axis(rc, {200, 200, 200, 0, 0, 0}, 1, 0, 0, 180, 50, 10, 2)
>>> # Rotate 180 degrees around the x-axis. Center of rotation is '{200, 200, 200, 0, 0, 0}'. Based on the center point of the rotation, the orientation of the TCP is changed together.
)pbdoc")
      .def("move_jb_clear", &PyCobot<T>::move_jb_clear, py::arg("response_collector"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false, R"pbdoc(
Initialize (Clear) the point list to be used in MoveJB.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_jb_add", &PyCobot<T>::move_jb_add, py::arg("response_collector"), py::arg("joint"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
This function adds the joint-angles used in MoveJB to the list.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
joint : numpy.ndarray(shape=(6, 1))
    Target joint angles (Unit: deg)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_jb_run", &PyCobot<T>::move_jb_run, py::arg("response_collector"), py::arg("speed"),
           py::arg("acceleration"), py::arg("timeout") = -1., py::arg("return_on_error") = false, R"pbdoc(
This function executes MoveJB using the points added in move_jb_add.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
speed : float
    Speed (Unit: deg/s)
acceleration : float
    Acceleration (Unit: deg/s^2)
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("move_jb2_clear", &PyCobot<T>::move_jb2_clear, py::arg("response_collector"), py::arg("timeout") = -1,
           py::arg("return_on_err") = false)
      .def("move_jb2_add", &PyCobot<T>::move_jb2_add, py::arg("response_collector"), py::arg("joint"), py::arg("speed"),
           py::arg("acceleration"), py::arg("blending_value"), py::arg("timeout") = -1,
           py::arg("return_on_err") = false)
      .def("move_jb2_run", &PyCobot<T>::move_jb2_run, py::arg("response_collector"), py::arg("timeout") = -1,
           py::arg("return_on_err") = false)
      .def("move_servo_j", &PyCobot<T>::move_servo_j, py::arg("response_collector"), py::arg("joint"), py::arg("t1"),
           py::arg("t2"), py::arg("gain"), py::arg("alpha"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false)
      .def("move_servo_l", &PyCobot<T>::move_servo_l, py::arg("response_collector"), py::arg("joint"), py::arg("t1"),
           py::arg("t2"), py::arg("gain"), py::arg("alpha"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false)
      .def("move_servo_t", &PyCobot<T>::move_servo_t, py::arg("response_collector"), py::arg("joint"), py::arg("t1"),
           py::arg("t2"), py::arg("compensation"), py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("move_speed_j", &PyCobot<T>::move_speed_j, py::arg("response_collector"), py::arg("joint"), py::arg("t1"),
           py::arg("t2"), py::arg("gain"), py::arg("alpha"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false)
      .def("move_speed_l", &PyCobot<T>::move_speed_l, py::arg("response_collector"), py::arg("joint"), py::arg("t1"),
           py::arg("t2"), py::arg("gain"), py::arg("alpha"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false)

      .def("set_tool_out", &PyCobot<T>::set_tool_out,
           py::arg("response_collector"), py::arg("voltage"), py::arg("signal_0"), py::arg("signal_1"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false)
      .def("gripper_rts_rhp12rn_select_mode", &PyCobot<T>::gripper_rts_rhp12rn_select_mode,
           py::arg("response_collector"), py::arg("conn_point"), py::arg("force"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false)
      .def("gripper_rts_rhp12rn_set_force_limit", &PyCobot<T>::gripper_rts_rhp12rn_set_force_limit,
           py::arg("response_collector"), py::arg("conn_point"), py::arg("limit"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false)
      .def("gripper_rts_rhp12rn_force_control", &PyCobot<T>::gripper_rts_rhp12rn_force_control,
           py::arg("response_collector"), py::arg("conn_point"), py::arg("target_force_ratio"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("gripper_rts_rhp12rn_position_control", &PyCobot<T>::gripper_rts_rhp12rn_position_control,
           py::arg("response_collector"), py::arg("conn_point"), py::arg("target_position_ratio"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)

      .def("gripper_koras_tooling_core_initialization", &PyCobot<T>::gripper_koras_tooling_core_initialization,
           py::arg("response_collector"), py::arg("conn_point"), py::arg("target_torque"), py::arg("target_speed"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("gripper_koras_tooling_vaccum_control", &PyCobot<T>::gripper_koras_tooling_vaccum_control,
           py::arg("response_collector"), py::arg("conn_point"), py::arg("vaccum_on_off"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("gripper_koras_tooling_finger_initialization", &PyCobot<T>::gripper_koras_tooling_finger_initialization,
           py::arg("response_collector"), py::arg("conn_point"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("gripper_koras_tooling_finger_open_close", &PyCobot<T>::gripper_koras_tooling_finger_open_close,
           py::arg("response_collector"), py::arg("conn_point"), py::arg("finger_open_close"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("gripper_koras_tooling_finger_goto", &PyCobot<T>::gripper_koras_tooling_finger_goto,
           py::arg("response_collector"), py::arg("conn_point"), py::arg("target_position"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)

      .def("gripper_inspire_humanoid_hand_initialization", &PyCobot<T>::gripper_inspire_humanoid_hand_initialization,
           py::arg("response_collector"), py::arg("reading_data_type"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("gripper_inspire_humanoid_hand_set_finger", &PyCobot<T>::gripper_inspire_humanoid_hand_set_finger,
           py::arg("response_collector"), py::arg("function"), py::arg("little"), py::arg("ring"), py::arg("middle"), py::arg("index"), py::arg("thumb1"), py::arg("thumb2"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)

      .def("task_load", &PyCobot<T>::task_load, py::arg("response_collector"), py::arg("program_name"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("task_play", &PyCobot<T>::task_play, py::arg("response_collector"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false)
      .def("task_stop", &PyCobot<T>::task_stop, py::arg("response_collector"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false)
      .def("task_pause", &PyCobot<T>::task_pause, py::arg("response_collector"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false)
      .def("task_resume", &PyCobot<T>::task_resume, py::arg("response_collector"), py::arg("collision"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("rt_script_onoff", &PyCobot<T>::rt_script_onoff, py::arg("response_collector"), py::arg("on"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("rt_script", &PyCobot<T>::rt_script, py::arg("response_collector"), py::arg("single_command"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("wait_for_task_loaded", &PyCobot<T>::wait_for_task_loaded, py::arg("response_collector"),
           py::arg("timeout") = -1., py::arg("return_on_error") = true)
      .def("wait_for_task_started", &PyCobot<T>::wait_for_task_started, py::arg("response_collector"),
           py::arg("timeout") = -1., py::arg("return_on_error") = true)
      .def("wait_for_task_finished", &PyCobot<T>::wait_for_task_finished, py::arg("response_collector"),
           py::arg("timeout") = -1., py::arg("return_on_error") = true)
      .def("eval", &PyCobot<T>::eval, py::arg("response_collector"), py::arg("script"), py::arg("timeout") = -1.,
           py::arg("return_on_error") = false, R"pbdoc(
A function that evaluates a script on the 'Cobot'.

This function sends the given script to the 'Cobot' for evaluation and waits for the response from the 'Cobot'. In the case of failure, the function returns 'Timeout' or 'Error'.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
script : str
    The script to be evaluated.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType
)pbdoc")
      .def("wait_until", &PyCobot<T>::wait_until, py::arg("response_collector"), py::arg("func"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("wait_until_ack_message", &PyCobot<T>::wait_until_ack_message, py::arg("response_collector"),
           py::arg("timeout") = -1., py::arg("return_on_error") = false)
      .def("wait_for_move_started", &PyCobot<T>::wait_for_move_started, py::arg("response_collector"),
           py::arg("timeout") = -1., py::arg("return_on_error") = true, R"pbdoc(
Wait until the motion is started.

More specifically, the program is waiting for the response message from control box ``info[motion_changed][X]`` where ``X`` is positive integer.

Parameters
----------
response_collector : ResponseCollector
    A collector object to accumulate and manage the response message.
timeout : float
    The maximum duration (in seconds) to wait for a response before timing out.
return_on_error : bool
    A boolean flag indicating whether the function should immediately return upon encountering an error.

Returns
-------
ReturnType

Examples
--------
>>> joint = np.array([0, 0, 0, 0, 0, 0])
>>> robot.move_j(rc, joint, 60, 80)
>>> rc = rc.error().throw_if_not_empty()
>>> if robot.wait_for_move_started(rc).type() == rb.ReturnType.Success:
>>>     robot.wait_for_move_finished(rc)
)pbdoc")
      .def("wait_for_move_finished", &PyCobot<T>::wait_for_move_finished, py::arg("response_collector"),
           py::arg("timeout") = -1., py::arg("return_on_error") = true)
      .def("disable_waiting_ack", &PyCobot<T>::disable_waiting_ack, py::arg("response_collector"))
      .def("enable_waiting_ack", &PyCobot<T>::enable_waiting_ack, py::arg("response_collector"))
      .def("flush", &PyCobot<T>::flush, py::arg("response_collector"));

  m.def("to_string", py::overload_cast<Response::Type>(&rb::podo::to_string), py::arg("type"));
}
