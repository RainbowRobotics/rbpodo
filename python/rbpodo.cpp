#include <Eigen/Core>

#include "pybind11/pybind11.h"

#include "cobot.hpp"
#include "cobot_data.hpp"
#include "common.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

void pybind11_data_type(py::module_& m);

PYBIND11_MODULE(_rbpodo, m) {
  m.doc() = R"pbdoc(
        Client Library for Rainbow Robotics' Cobot RB-Series
        ----------------------------------------------------

        .. currentmodule:: rbpodo

        .. autosummary::
           :toctree: _generate

           Cobot
           CobotData
    )pbdoc";

  pybind11_data_type(m);
  pybind11_response(m);
  pybind11_error_code(m);

  pybind11_cobot<_macro::Sync>(m);
  pybind11_cobot_data<_macro::Sync>(m);

  auto asyncio_m = m.def_submodule("asyncio", "Asyncio version of rbpodo");
  pybind11_cobot<_macro::Async>(asyncio_m);
  pybind11_cobot_data<_macro::Async>(asyncio_m);

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}