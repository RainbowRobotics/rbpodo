/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#pragma once

#include <iostream>
#include "pybind11/functional.h"
#include "pybind11/numpy.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pybind11/eigen.h"

namespace py = pybind11;

namespace rb::podo::_macro {

class Sync {};

class Async {};

template <typename F, typename... Args>
auto async_wrapper_helper(F func, Args... args) {
  py::object loop = py::module_::import("asyncio.events").attr("get_event_loop")();
  return loop.attr("run_in_executor")(py::none(), py::cpp_function([=] {
                                        py::gil_scoped_release guard{};
                                        return std::bind(func, args...)();
                                      }));
}

}  // namespace rb::podo::_macro

#define END(...) END_(__VA_ARGS__)
#define END_(...) __VA_ARGS__##_END

#define PARAMS_LOOP_0(type_, name_) PARAMS_LOOP_BODY(type_, name_) PARAMS_LOOP_A
#define PARAMS_LOOP_A(type_, name_) , PARAMS_LOOP_BODY(type_, name_) PARAMS_LOOP_B
#define PARAMS_LOOP_B(type_, name_) , PARAMS_LOOP_BODY(type_, name_) PARAMS_LOOP_A
#define PARAMS_LOOP_0_END
#define PARAMS_LOOP_A_END
#define PARAMS_LOOP_B_END
#define PARAMS_LOOP_BODY(type_, name_) type_ name_

#define VAR_LOOP_0(type_, name_) VAR_LOOP_BODY(type_, name_) VAR_LOOP_A
#define VAR_LOOP_A(type_, name_) , VAR_LOOP_BODY(type_, name_) VAR_LOOP_B
#define VAR_LOOP_B(type_, name_) , VAR_LOOP_BODY(type_, name_) VAR_LOOP_A
#define VAR_LOOP_0_END
#define VAR_LOOP_A_END
#define VAR_LOOP_B_END
#define VAR_LOOP_BODY(type_, name_) name_

#define ASYNC_FUNC_WRAPPER_HELPER(func_)                                              \
  if constexpr (std::is_same_v<T, _macro::Sync>) {                                    \
    py::gil_scoped_release g{};                                                       \
    return func_();                                                                   \
  } else {                                                                            \
    py::object loop = py::module_::import("asyncio.events").attr("get_event_loop")(); \
    return loop.attr("run_in_executor")(py::none(), py::cpp_function([=] {            \
                                          py::gil_scoped_release guard{};             \
                                          return func_();                             \
                                        }));                                          \
  }

#define ASYNC_FUNC_WRAPPER(class_, func_, ...)                               \
  auto func_(END(PARAMS_LOOP_0 __VA_ARGS__)) {                               \
    auto _func = [=] {                                                       \
      return static_cast<class_&>(*this).func_(END(VAR_LOOP_0 __VA_ARGS__)); \
    };                                                                       \
    ASYNC_FUNC_WRAPPER_HELPER(_func)                                         \
  }

#define ASYNC_FUNC_WRAPPER_RC(class_, func_, ...)                                 \
  auto func_(ResponseCollector* _rc END(PARAMS_LOOP_A __VA_ARGS__)) {             \
    auto _func = [=] {                                                            \
      return static_cast<class_&>(*this).func_(*_rc END(VAR_LOOP_A __VA_ARGS__)); \
    };                                                                            \
    ASYNC_FUNC_WRAPPER_HELPER(_func)                                              \
  }
