#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace rb::podo;
using namespace std::chrono_literals;

int main() {
  try {
    auto robot = Cobot("10.0.2.7");
    auto rc = ResponseCollector();
    int count;

    rc.set_callback([](const Response& res) {
      if (res.type() == Response::Type::Error) {
        std::cerr << "error" << std::endl;
        exit(-1);
      }
    });

    robot.eval(rc, "var count = 0");

    robot.rt_script_onoff(rc, true);
    robot.rt_script(rc, "count += 1");

    auto c = std::chrono::steady_clock::now();
    for (int i = 0; i < 500; i++) {
      c += 1000ms;
      std::this_thread::sleep_until(c);

      std::string tmp;
      robot.print_variable(rc, "count", tmp);
      int tmp_count = atoi(tmp.c_str());

      if(i != 0) {
        std::cout << "[" << i << "] delta count = " << tmp_count - count << std::endl;
      }
      count = tmp_count;
    }

    robot.rt_script_onoff(rc, false);

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}