/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#pragma once

#include "common.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "rbpodo/cobot_data.hpp"

namespace py = pybind11;
using namespace rb::podo;

template <typename T>
class PyCobotData : public CobotData {
 public:
  PyCobotData(const std::string& address, int port) : CobotData(address, port) {}

  ASYNC_FUNC_WRAPPER(CobotData, request_data, (double, timeout))

  [[nodiscard]] bool is_alive() const override {
    py::gil_scoped_acquire g{};
    bool interrupt = (PyErr_CheckSignals() == 0);
    if (!interrupt) {
      throw py::error_already_set();
    }
    return interrupt;
  }
};

template <typename T>
void pybind11_cobot_data(py::module_& m) {
  py::class_<PyCobotData<T>>(m, "CobotData")
      .def(py::init<const std::string&, int>(), py::arg("address"), py::arg("port") = kDataPort)
      .def("request_data", &PyCobotData<T>::request_data, py::arg("timeout") = -1.0);
}