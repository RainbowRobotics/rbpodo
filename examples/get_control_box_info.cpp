#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace rb;

int main() {
  try {
    // Make connection
    podo::Cobot robot("10.0.2.7");
    podo::ResponseCollector rc;

    podo::ControlBoxInfo cbi;
    robot.get_control_box_info(rc, cbi);
    rc.error().throw_if_not_empty();

    std::cout << "Contorl Box Info: " << cbi.str() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}