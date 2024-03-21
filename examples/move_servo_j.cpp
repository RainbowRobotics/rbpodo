#include <iostream>

#include "rbpodo/rbpodo.hpp"

using namespace rb;
using namespace std::chrono_literals;

int main() {
  try {
    auto robot = podo::Cobot("10.0.2.7");
    auto rc = podo::ResponseCollector();

    robot.set_operation_mode(rc, podo::OperationMode::Real);
    robot.set_speed_bar(rc, 1.0);
    rc = rc.error().throw_if_not_empty();

    // Go home pose
    robot.move_j(rc, {0, 0, 0, 0, 0, 0}, 50, 100);
    if (robot.wait_for_move_started(rc, 0.1).is_success())
      robot.wait_for_move_finished(rc);

    // move_servo_j
    robot.disable_waiting_ack(rc);
    for (int i = 0; i < 1000; i++) {
      robot.move_servo_j(rc, {(double)i * 90. / 1000., 0, 0, 0, 0, 0}, 0.01, 0.1, 1.0, 1.0);
      std::this_thread::sleep_for(5ms);
    }
    robot.move_speed_j(rc, {0, 0, 0, 0, 0, 0}, 0.01, 0.1, 1.0, 1.0);
    robot.enable_waiting_ack(rc);
    robot.wait_for_move_finished(rc);
    rc.clear();

    robot.move_j(rc, {0, 0, 0, 0, 0, 0}, 50, 100);
    if (robot.wait_for_move_started(rc, 0.1).is_success())
      robot.wait_for_move_finished(rc);

    rc = rc.error().throw_if_not_empty();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}