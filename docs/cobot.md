# Cobot Class

> :info: This page is under actively construction. You can find the hints in header files.

> :warning: **Note:** These APIs are not thread-safe.

## ``Cobot`` constructor

```c++
Cobot(const std::string& address, int port = kCommandPort);
```

## get_control_box_info

```c++
ReturnType get_control_box_info(ResponseCollector& response_collector, ControlBoxInfo& info, double timeout = -1.,
                                bool return_on_error = false);
```

## get_robot_state

```c++
ReturnType get_robot_state(ResponseCollector& response_collector, RobotState& robot_state, double timeout = -1.,
                           bool return_on_error = false);
```

## activate

```c++
ReturnType activate(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = true);
```

## shutdown

```c++
ReturnType shutdown(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false);
```

## get_system_variable

```c++
ReturnType get_system_variable(ResponseCollector& response_collector, SystemVariable sv, double& out,
                               double timeout = -1., bool return_on_error = false);
```

## print_variable

```c++
ReturnType print_variable(ResponseCollector& response_collector, const std::string& variable_name, std::string& out,
                          double timeout = -1., bool return_on_error = false);
```

## set_payload_info

```c++
ReturnType set_payload_info(ResponseCollector& response_collector, double weight, double com_x, double com_y,
                            double com_z, double timeout = -1., bool return_on_error = false);
```

## get_tcp_info

```c++
ReturnType get_tcp_info(ResponseCollector& response_collector, PointRef point, double timeout = -1.,
                        bool return_on_error = false);
```

## get_tfc_info

```c++
ReturnType get_tfc_info(ResponseCollector& response_collector, PointRef point, double timeout = -1.,
                        bool return_on_error = false);
```

## set_tcp_info

```c++
ReturnType set_tcp_info(ResponseCollector& response_collector, PointConstRef point, double timeout = -1.,
                        bool return_on_error = false);
```

## set_user_coordinate

```c++
ReturnType set_user_coordinate(ResponseCollector& response_collector, int id, PointConstRef point, double timeout = -1.,
                               bool return_on_error = false);
```

## set_operation_mode

```c++
ReturnType set_operation_mode(ResponseCollector& response_collector, OperationMode mode, double timeout = -1.,
                              bool return_on_error = false);
```

## set_speed_bar

```c++
ReturnType set_speed_bar(ResponseCollector& response_collector, double speed, double timeout = -1.,
                         bool return_on_error = false);
```

Set the overall speed control bar. (bottom speed control bar in UI).

### Parameters

- speed: desired speed control bar position (0~1)

### Example

```c++
auto robot = rb::podo::Cobot("10.0.2.7");
auto response_collector = rb::podo::ResponseCollector();
robot.set_speed_bar(response_collector, 1.0);  // 100% speed
```

## set_freedrive_mode

```c++
ReturnType set_freedrive_mode(ResponseCollector& response_collector, bool on, double timeout = -1.,
                              bool return_on_error = false);
```

## set_collision_onoff

```c++
ReturnType set_collision_onoff(ResponseCollector& response_collector, bool on, double timeout = -1.,
                               bool return_on_error = false);
```

## set_collision_threshold

```c++
ReturnType set_collision_threshold(ResponseCollector& response_collector, double threshold, double timeout = -1.,
                                   bool return_on_error = false);
```

## set_collision_mode

```c++
ReturnType set_collision_mode(ResponseCollector& response_collector, CollisionMode mode, double timeout = -1.,
                              bool return_on_error = false);
```

## set_collision_after

```c++
ReturnType set_collision_after(ResponseCollector& response_collector, CollisionReactionMode mode, double timeout = -1.,
                               bool return_on_error = false);
```

## set_speed_multiplier

```c++
ReturnType set_speed_multiplier(ResponseCollector& response_collector, double multiplier, double timeout = -1.,
                                bool return_on_error = false);
```

## set_acc_multiplier

```c++
ReturnType set_acc_multiplier(ResponseCollector& response_collector, double multiplier, double timeout = -1.,
                              bool return_on_error = false);
```

## set_speed_acc_j

```c++
ReturnType set_speed_acc_j(ResponseCollector& response_collector, double speed, double acceleration, double timeout = -1.,
                           bool return_on_error = false);
```

## set_speed_acc_l

```c++
ReturnType set_speed_acc_l(ResponseCollector& response_collector, double speed, double acceleration, double timeout = -1.,
                           bool return_on_error = false);
```

Sets fixed linear velocity/acceleration for L-series motions (MoveL, MovePB, MoveLB, MoveITPL).

> [!CAUTION]
> The value set in this function returns to the default value after the program ends.  
> During program flow, the value set in this function is maintained until this function is called again

### Parameters

- speed: linear velocity for L-series motion (unit: deg/s). Ignore if value is negative.
- acceleration: linear acceleration for L-series motion (unit: deg/s^2). Ignore if value is negative.

## set_box_dout

```c++
ReturnType set_box_dout(ResponseCollector& response_collector, int port, DigitalIOMode mode, double timeout = -1.,
                        bool return_on_error = false);
```

## set_box_aout

```c++
ReturnType set_box_aout(ResponseCollector& response_collector, int port, double voltage, double timeout = -1.,
                        bool return_on_error = false);
```

## set_box_dout_toggle

```c++
ReturnType set_box_dout_toggle(ResponseCollector& response_collector, int port, double timeout = -1.,
                               bool return_on_error = false);
```

## calc_fk_tcp

```c++
ReturnType calc_fk_tcp(ResponseCollector& response_collector, PointRef point, double j0, double j1, double j2,
                       double j3, double j4, double j5, double timeout = -1., bool return_on_error = false);
```

## calc_fk_tcp

```c++
ReturnType calc_fk_tcp(ResponseCollector& response_collector, PointRef point, JointConstRef joint, double timeout = -1.,
                       bool return_on_error = false);
```

## move_l

```c++
ReturnType move_l(ResponseCollector& response_collector, PointConstRef point, double speed, double acceleration,
                  double timeout = -1., bool return_on_error = false);
```

## move_l_rel

```c++
ReturnType move_l_rel(ResponseCollector& response_collector, PointConstRef point, double speed, double acceleration,
                      ReferenceFrame frame, double timeout = -1., bool return_on_error = false);
```

## move_j

```c++
ReturnType move_j(ResponseCollector& response_collector, JointConstRef joint, double speed, double acceleration,
                  double timeout = -1., bool return_on_error = false);
```

## move_jl

```c++
ReturnType move_jl(ResponseCollector& response_collector, PointConstRef point, double speed, double acceleration,
                   double timeout = -1., bool return_on_error = false);
```

## move_pb_clear

```c++
ReturnType move_pb_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false);
```

## move_pb_add

```c++
ReturnType move_pb_add(ResponseCollector& response_collector, PointConstRef point, double speed, BlendingOption option,
                       double blending_value, double timeout = -1., bool return_on_error = false);
```

## move_pb_run

```c++
ReturnType move_pb_run(ResponseCollector& response_collector, double acceleration, MovePBOption option,
                       double timeout = -1., bool return_on_error = false);
```

## move_itpl_clear

```c++
ReturnType move_itpl_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false);
```

## move_itpl_add

```c++
ReturnType move_itpl_add(ResponseCollector& response_collector, PointConstRef point, double speed, double timeout = -1.,
                         bool return_on_error = false);
```

## move_itpl_run

```c++
ReturnType move_itpl_run(ResponseCollector& response_collector, double acceleration, MoveITPLOption option,
                         double timeout = -1., bool return_on_error = false);
```

## move_lc_clear

```c++
ReturnType move_lc_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false);
```

## move_lc_add

```c++
ReturnType move_lc_add(ResponseCollector& response_collector, PointConstRef point, double speed,
                       MoveLCProperty property, double timeout = -1., bool return_on_error = false);
```

## move_lc_run

```c++
ReturnType move_lc_run(ResponseCollector& response_collector, double acceleration, MoveLCOption option,
                       double timeout = -1., bool return_on_error = false);
```

## move_lb_clear

```c++
ReturnType move_lb_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false);
```

## move_lb_add

```c++
ReturnType move_lb_add(ResponseCollector& response_collector, PointConstRef point, double blend_distance,
                       double timeout = -1., bool return_on_error = false);
```

## move_lb_run

```c++
ReturnType move_lb_run(ResponseCollector& response_collector, double speed, double acceleration, MoveLBOption option,
                       double timeout = -1., bool return_on_error = false);
```

## move_c_points

```c++
ReturnType move_c_points(ResponseCollector& response_collector, PointConstRef via_point, PointConstRef target_point,
                         double speed, double acceleration, MoveCOrientationOption option, double timeout = -1.,
                         bool return_on_error = false);
```

## move_c_axis

```c++
ReturnType move_c_axis(ResponseCollector& response_collector, PointConstRef center, double x_axis, double y_axis,
                       double z_axis, double angle, double speed, double acceleration, MoveCRotationOption option,
                       double timeout = -1., bool return_on_error = false);
```

## move_jb_clear

```c++
ReturnType move_jb_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false);
```

## move_jb_add

```c++
ReturnType move_jb_add(ResponseCollector& response_collector, JointConstRef joint, double timeout = -1.,
                       bool return_on_error = false);
```

## move_jb_run

```c++
ReturnType move_jb_run(ResponseCollector& response_collector, double speed, double acceleration, double timeout = -1.,
                       bool return_on_error = false);
```

## move_jb2_clear

```c++
ReturnType move_jb2_clear(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false);
```

## move_jb2_add

```c++
ReturnType move_jb2_add(ResponseCollector& response_collector, JointConstRef joint, double speed, double acceleration,
                        double blending_value, double timeout = -1., bool return_on_error = false);
```

## move_jb2_run

```c++
ReturnType move_jb2_run(ResponseCollector& response_collector, double timeout = -1., bool return_on_error = false);
```

## move_servo_j

```c++
ReturnType move_servo_j(ResponseCollector& response_collector, JointConstRef joint, double t1, double t2, double gain,
                        double alpha, double timeout = -1., bool return_on_error = false);
```

## move_servo_l

```c++
ReturnType move_servo_l(ResponseCollector& response_collector, PointConstRef point, double t1, double t2, double gain,
                        double alpha, double timeout = -1., bool return_on_error = false);
```

## move_servo_t

```c++
ReturnType move_servo_t(ResponseCollector& response_collector, JointConstRef joint, double t1, double t2,
                        int compensation, double timeout = -1., bool return_on_error = false);
```

## move_speed_j

```c++
ReturnType move_speed_j(ResponseCollector& response_collector, JointConstRef joint, double t1, double t2, double gain,
                        double alpha, double timeout = -1., bool return_on_error = false);
```

## move_speed_l

```c++
ReturnType move_speed_l(ResponseCollector& response_collector, PointConstRef point, double t1, double t2, double gain,
                        double alpha, double timeout = -1., bool return_on_error = false);
```

## eval

```c++
ReturnType eval(ResponseCollector& response_collector, const std::string& script, double timeout = -1.,
                bool return_on_error = false);
```

## disable_waiting_ack

```c++
bool disable_waiting_ack(ResponseCollector& response_collector);
```

## enable_waiting_ack

```c++
bool enable_waiting_ack(ResponseCollector& response_collector);
```

## flush

```c++
void flush(ResponseCollector& response_collector);
```

## Helper functions

```c++
template <typename T>
ReturnType wait_for_printed_value(ResponseCollector& response_collector, T& out, const std::regex& pattern,
                                  double timeout = -1, bool return_on_error = true);

ReturnType wait_until(ResponseCollector& response_collector, const std::function<bool(const Response&)>& func,
                      double timeout = -1., bool return_on_error = false);

ReturnType wait_until_ack_message(ResponseCollector& response_collector, double timeout = -1.,
                                  bool return_on_error = false);

ReturnType wait_for_move_started(ResponseCollector& response_collector, double timeout = -1.,
                                 bool return_on_error = true);

ReturnType wait_for_move_finished(ResponseCollector& response_collector, double timeout = -1.,
                                bool return_on_error = true);
```