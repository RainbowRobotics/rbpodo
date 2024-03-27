#include <iostream>

#include "rbpodo/rbpodo.hpp"

using namespace rb;

int main() {
  try {
    auto robot = podo::Cobot("10.0.2.7");
    auto rc = podo::ResponseCollector();

    robot.set_operation_mode(rc, rb::podo::OperationMode::Simulation);
    rc = rc.error().throw_if_not_empty();

    robot.move_pb_clear(rc);
    robot.move_pb_add(rc, {100, 200, 200, 90, 0, 0}, 200.0, rb::podo::BlendingOption::Ratio, 0.5);
    robot.move_pb_add(rc, {300, 300, 400, 0, 0, 0}, 400.0, rb::podo::BlendingOption::Ratio, 0.5);
    robot.move_pb_add(rc, {0, 200, 400, 90, 0, 0}, 200.0, rb::podo::BlendingOption::Ratio, 0.5);

    // **Important**
    // Before you start move, flush buffer to response collector to avoid unexpected behavior in 'wait' function
    robot.flush(rc);
    rc = rc.error().throw_if_not_empty();
    rc.clear();

    robot.move_pb_run(rc, 800, rb::podo::MovePBOption::Intended);
    auto res = robot.wait_for_move_started(rc, 0.5);
    if (res.is_success()) {
      robot.wait_for_move_finished(rc);
    } else {
      std::cerr << "Not move" << std::endl;
      std::cerr << res << std::endl;
    }
    // If there is any error during above process, throw exception error
    rc.error().throw_if_not_empty();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}