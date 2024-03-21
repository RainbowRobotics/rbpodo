#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace rb;

int main() {
  try {
    // Make connection
    auto robot = podo::Cobot("10.0.2.7");
    auto rc = podo::ResponseCollector();

    double out;
    robot.get_system_variable(rc, podo::SystemVariable::SD_J1_REF, out);
    rc = rc.error().throw_if_not_empty();

    std::cout << out << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}