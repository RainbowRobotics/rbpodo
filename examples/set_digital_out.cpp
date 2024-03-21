#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace rb;

int main() {
  try {
    // Make connection
    auto robot = podo::Cobot("10.0.2.7");
    auto rc = podo::ResponseCollector();

    robot.set_box_dout(rc, 0, podo::DigitalIOMode::High);
    rc = rc.error().throw_if_not_empty();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}