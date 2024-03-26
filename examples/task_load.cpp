#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace std;
using namespace rb::podo;

int main() {
  try {
    auto robot = Cobot("10.0.2.7");
    auto rc = ResponseCollector();

    robot.task_load(rc, "default");
    if(robot.wait_for_task_loaded(rc).type() == ReturnType::Success) {
      std::cout << "'default' wsl exists" << std::endl;
    } else {
      std::cerr << "Error" << std::endl;
    }
    rc = rc.error().throw_if_not_empty();

    robot.task_load(rc, "default_does_not_exist");
    if(robot.wait_for_task_loaded(rc).type() == ReturnType::Error) {
      std::cout << "'default_dose_not_exist' wsl does not exist" << std::endl;
    }
    rc = rc.error().throw_if_not_empty();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return 0;
}