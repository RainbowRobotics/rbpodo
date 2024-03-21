#include <iostream>

#include <Eigen/Core>
#include "rbpodo/rbpodo.hpp"

using namespace rb;
using namespace std::chrono_literals;

Eigen::IOFormat io_format({Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"});

int main() {
  try {
    auto robot = podo::Cobot<rb::podo::EigenVector>("10.0.2.7");
    auto data_channel = podo::CobotData("10.0.2.7");
    auto rc = podo::ResponseCollector();

    robot.set_operation_mode(rc, podo::OperationMode::Real);
    robot.set_speed_bar(rc, 1.0);
    rc = rc.error().throw_if_not_empty();

    // Go home pose
    robot.move_j(rc, Eigen::Matrix<double, 6, 1>({0, 0, 0, 0, 0, 0}), 50, 100);
    if (robot.wait_for_move_started(rc, 0.1).is_success())
      robot.wait_for_move_finished(rc);

    auto stime = std::chrono::steady_clock::now();

    Eigen::Vector<double, 6> target_q = Eigen::Vector<double, 6>::Zero();
    Eigen::Vector<double, 6> current_q;
    Eigen::Vector<double, 6> target_torque;

    auto data = data_channel.request_data();
    current_q = Eigen::Map<Eigen::Vector<float, 6>>(data->sdata.jnt_ang).cast<double>();

    Eigen::Array<double, 6, 1> K;
    K << 0.6, 0.6, 0.4, 0.3, 0.2, 0.2;

    // move_servo_j
    robot.disable_waiting_ack(rc);
    for (int i = 0; i < 4000; i++) {
      stime += std::chrono::milliseconds(5);
      std::this_thread::sleep_until(stime);

      data = data_channel.request_data();
      current_q = Eigen::Map<Eigen::Vector<float, 6>>(data->sdata.jnt_ang).cast<double>();
      target_torque = K * (target_q - current_q).array();

      robot.move_servo_t(rc, target_torque, 0.01, 0.1, 1);

      std::cout << "--------------------------" << std::endl;
      std::cout << "angle:  " << current_q.transpose().format(io_format) << std::endl;
      std::cout << "torque: " << Eigen::Map<Eigen::Vector<float, 6>>(data->sdata.jnt_cur).transpose().format(io_format)
                << std::endl;
      std::cout << "target: " << target_torque.transpose().format(io_format) << std::endl;
    }
    robot.enable_waiting_ack(rc);
    robot.wait_for_move_finished(rc);

    robot.move_j(rc, Eigen::Matrix<double, 6, 1>({0, 0, 0, 0, 0, 0}), 50, 100);
    if (robot.wait_for_move_started(rc, 0.1).is_success())
      robot.wait_for_move_finished(rc);

    rc = rc.error().throw_if_not_empty();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}