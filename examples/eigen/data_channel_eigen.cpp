#include <iostream>

#include <Eigen/Core>
#include "rbpodo/rbpodo.hpp"

int main() {
  auto data_channel = rb::podo::CobotData("10.0.2.7");

  auto timer = rb::podo::Timer();
  auto data = data_channel.request_data();
  std::cout << timer.get_duration() << "s" << std::endl;
  if (data) {
    // Joint reference
    Eigen::Vector<float, 6> jnt;
    jnt = Eigen::Map<Eigen::Vector<float, 6>>(data->sdata.jnt_ref);
    std::cout << jnt << std::endl;
  }
  return 0;
}