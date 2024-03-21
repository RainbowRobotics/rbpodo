#include <iostream>

#include "rbpodo/rbpodo.hpp"

using namespace std::chrono_literals;
using namespace rb;

int main() {
  try {
    auto robot = podo::Cobot("10.0.2.7");
    auto rc = podo::ResponseCollector();

    robot.set_operation_mode(rc, podo::OperationMode::Simulation);

    std::cout << "Set speed bar value to 1.0" << std::endl;
    robot.set_speed_bar(rc, 1.0);
    rc = rc.error().throw_if_not_empty();

    std::this_thread::sleep_for(1s);

    std::cout << "Set speed bar value to 0.5" << std::endl;
    robot.set_speed_bar(rc, 0.5);
    rc = rc.error().throw_if_not_empty();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}