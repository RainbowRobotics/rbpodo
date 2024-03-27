# Getting Started

## Basic example

```c++
#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace rb;

int main() {
  try {
    // Make connection
    podo::Cobot robot("10.0.2.7");
    podo::ResponseCollector rc;

    robot.set_operation_mode(rc, podo::OperationMode::Simulation);
    robot.set_speed_bar(rc, 0.5);

    robot.flush(rc);
    // Move robot in joint space
    robot.move_j(rc, {100, 0, 0, 0, 0, 0}, 200, 400);
    if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success) {
      robot.wait_for_move_finished(rc);
    }
    // If there is any error during above process, throw exception error
    rc.error().throw_if_not_empty();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
```

## Move operation

We provide lots of ``move`` operations. You can find them in [api reference](./api_reference.md) page.

```c++
robot.move_pb_clear(rc);
robot.move_pb_add(rc, {100, 200, 200, 90, 0, 0}, 200.0, rb::podo::BlendingOption::Ratio, 0.5);
robot.move_pb_add(rc, {300, 300, 400, 0, 0, 0}, 400.0, rb::podo::BlendingOption::Ratio, 0.5);
robot.move_pb_add(rc, {0, 200, 400, 90, 0, 0}, 200.0, rb::podo::BlendingOption::Ratio, 0.5);
robot.move_pb_run(rc, 800, rb::podo::MovePBOption::Intended);
rc = rc.error().throw_if_not_empty();
```

## Set program mode

```c++
robot.set_operation_mode(rc, rb::podo::OperationMode::Simulation);
```

## Set configuration

```c++
robot.set_speed_bar(rc, 1.0);
```

## Calculate kinematics

Using ``Cobot``, you can not only send command to robot, but also calculate kinematics.

```c++
rb::podo::Point p1, p2;
// Calculate TCP position and orientation when q = [20, 20, 20, 20, 20, 20] and store it in 'p1'
robot.calc_fk_tcp(rc, p1, 20, 20, 20, 20, 20, 20);
robot.calc_fk_tcp(rc, p2, Eigen::Vector<double, 6>(10, 10, 10, 10, 10, 10));
```

## Response handling

Many of the ``rbpodo`` APIs are asynchronous. For example, ``move_j`` returns even if the movement is not yet finished.
It returns when the robot sends an ACK message, indicating that the robot has received the command.
``Response`` refers to the message from the robot via ``command channel``. ``ResponseCollector`` is like a queue that store ``Response``.
Our APIs receive the storage ``ResponseCollector`` and store messages into that storage during that function performs.
If you want to know more details please check [here](./notes.md).

```c++
auto rc = rb::podo::ResponseCollector();

auto error_response = rc.error();  // Find errors in ResponseCollector
error_rc.throw_if_not_empty();  // Throw std::excpetion when the 'rc' queue is not empty

std::cout << rc << std::end;  // [{type: info, category: "...", msg: "..."}, ...]
```

## Get data (system state) via ``data channel``

We can request data (system state; related class is called ``SystemState``) via ``data channel``.
We can make a connection using ``CobotData`` class.

```c++
#include <iostream>

#include "rbpodo/rbpodo.hpp"

int main() {
  auto data_channel = rb::podo::CobotData("10.0.2.7");

  auto data = data_channel.request_data();
  if (data) {
    // Joint reference
    Eigen::Map<Eigen::Vector<float, 6>>(data->sdata.jnt_ref).cast<double>();
  }
  return 0;
}
```