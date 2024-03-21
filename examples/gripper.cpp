#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace std::chrono_literals;
using namespace rb;

int main() {
  try {
    // Make connection
    auto robot = podo::Cobot("10.0.2.7");
    auto rc = podo::ResponseCollector();

        robot.gripper_rts_rhp12rn_select_mode(rc, podo::GripperConnectionPoint::ToolFlange_Advanced, true);
        std::this_thread::sleep_for(0.5s);

        robot.gripper_rts_rhp12rn_force_control(rc, podo::GripperConnectionPoint::ToolFlange_Advanced, 100);
        std::this_thread::sleep_for(2s);

        robot.gripper_rts_rhp12rn_force_control(rc, podo::GripperConnectionPoint::ToolFlange_Advanced, -100);
        std::this_thread::sleep_for(2s);

        robot.gripper_rts_rhp12rn_force_control(rc, podo::GripperConnectionPoint::ToolFlange_Advanced, 0);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}