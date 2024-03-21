#include <iostream>

#include "rbpodo/rbpodo.hpp"

int main() {
  auto data_channel = rb::podo::CobotData("10.0.2.7");

  auto timer = rb::podo::Timer();
  auto data = data_channel.request_data();
  std::cout << timer.get_duration() << "s" << std::endl;
  if (data) {
    // Joint reference
    for (float i : data->sdata.jnt_ref) {
      std::cout << i << " ";
    }
    std::cout << std::endl;
  }
  return 0;
}