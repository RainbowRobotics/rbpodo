#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace rb;

int main() {
  // Make connection
  podo::Cobot robot("10.0.2.7");
  podo::ResponseCollector rc;

  robot.set_operation_mode(rc, podo::OperationMode::Simulation);
  robot.set_speed_bar(rc, 0.5);

  robot.flush(rc);

  // Raise an error
  robot.move_j(rc, {180 * 10, 0, 0, 0, 0, 0}, 10, 10);
  if (robot.wait_for_move_started(rc, 0.1).is_success()) {
    robot.wait_for_move_finished(rc);
  }
  for (const auto& err : rc.error()) {
    if(err.category() == "code") {
      int error_code = stoi(err.msg());
      std::cerr << "Error Code: " << error_code << ", Message: " << podo::ErrorCodeMessage[error_code].en << std::endl;
    }
  }
  return 0;
}