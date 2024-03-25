#include <iostream>
#include <array>
#include "rbpodo/rbpodo.hpp"

using namespace rb::podo;

int main() {
  try {
    auto robot = Cobot("10.0.2.7");
    auto rc = ResponseCollector();

    std::array<double, 6> pnt{};

    robot.get_tcp_info(rc, pnt);
    std::cout << "get_tcp_info() : ";
    for(const auto& e : pnt) {
      std::cout << e << " ";
    }
    std::cout << std::endl;

    robot.get_tfc_info(rc, pnt);
    std::cout << "get_tfc_info() : ";
    for(const auto& e : pnt) {
      std::cout << e << " ";
    }
    std::cout << std::endl;

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}