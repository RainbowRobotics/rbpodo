#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace rb;

int main() {
  try {
    // Make connection
    auto robot = podo::Cobot("10.0.2.7");
    auto rc = podo::ResponseCollector();

    std::cout << "Activating... " << std::endl;
    robot.activate(rc);
    rc.error().throw_if_not_empty();
    std::cout << "  done" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Shutdown" << std::endl;
    robot.shutdown(rc);
    rc.error().throw_if_not_empty();

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}