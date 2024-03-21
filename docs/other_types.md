# Other Types

## Primitive types

### Joint and Point

``Joint`` class represents a 6-dimensional vector (in R^6) that stores values related to the joints of a robot,
such as joint angle, current,and joint reference values. The ``Point`` class denotes a position and orientation
in space, also as a 6-dimensional vector. The first three dimensions represent the position (x, y, z),
while the last three dimensions represent the Euler ZYX angles (rx, ry, rz). Note that, although rotation is
represented as Euler angle ZYX, their vector representation adheres to the conventional xyz ordering.
Although both classes fundamentally store six floating-point numbers as 6-dimensional vectors,
they are distinguished to clarify their distinct meanings.

``rbpodo`` class supports Eigen vector types to make integrating with Eigen library based code easier.
You can switch to Eigen vector by including Eigen (3.3.7 or later) before ``rbpodo``

```c++
#include <Eigen/Core>
#include <rpbodo/rbpodo.hpp>
```

and then construct the Cobot class with ``rb::podo::EigenVector``.

```c++
auto robot = podo::Cobot<rb::podo::EigenVector>("10.0.2.7");
auto rc = podo::ResponseCollector();
```

Now, you can interact with ``Cobot`` via Eigen vector.

```c++
Eigen::Matrix<double, 6, 1> target_q;
target_q << 180, 0, 0, 0, 0, 0;
robot.move_j(rc, target_q, 50, 100);
```

Types of ``Vector``, ``Point``, and their references are aliased as follows when Eigen library is used,

```c++
class EigenVector {
 public:
  using Joint = Eigen::Vector<double, 6>;
  using Point = Eigen::Vector<double, 6>;
  using JointRef = Eigen::Ref<Joint>;
  using PointRef = Eigen::Ref<Point>;
  using JointConstRef = const Eigen::Ref<const Joint>&;
  using PointConstRef = const Eigen::Ref<const Point>&;
  ...
}
```

In environments where the Eigen library is not used, these types are aliased as follows, using the standard array:

```c++
class StandardVector {
 public:
  using Joint = std::array<double, 6>;
  using Point = std::array<double, 6>;
  using JointRef = Joint&;
  using PointRef = Point&;
  using JointConstRef = const Joint&;
  using PointConstRef = const Point&;
  ...
}
```

> [!INFO]
> Unfortunately, we do not support Eigen vector on ``CobotData`` class.

## Enum types

### System variables

Enum ``SystemVariable`` is designed to specify system value that can be requested
through a function ``cobot.get_system_variable(...)``.

For example, you can use it in C++ like below:

```c++
double out;
robot.get_system_variable(rc, podo::SystemVariable::SD_J1_REF, out);
```

Or, in Python

```python
import rbpodo as rb

[_, out] = robot.get_system_variable(rc, rb.SystemVariable.SD_J1_REF)
```

```c++
enum class SystemVariable {
  /// A variable representing time. (Unit: Second)
  /// @note It can be changed to the desired value in the Set function.
  /// @note Used as a timer function.
  /// @note Time automatically increments with the flow of program time.
  SD_TIME,
  SD_TIMER_0,
  SD_TIMER_1,
  SD_TIMER_2,
  SD_TIMER_3,
  SD_TIMER_4,
  SD_TIMER_5,
  SD_TIMER_6,
  SD_TIMER_7,
  SD_TIMER_8,
  SD_TIMER_9,

  /// Joint reference angles. (Unit: degree)
  SD_J0_REF,
  SD_J1_REF,
  SD_J2_REF,
  SD_J3_REF,
  SD_J4_REF,
  SD_J5_REF,

  /// Joint encoder angles. (Unit: degree)
  SD_J0_ANG,
  SD_J1_ANG,
  SD_J2_ANG,
  SD_J3_ANG,
  SD_J4_ANG,
  SD_J5_ANG,

  /// Joint encoder velocity. (Unit: degree/s)
  SD_J0_VEL,
  SD_J1_VEL,
  SD_J2_VEL,
  SD_J3_VEL,
  SD_J4_VEL,
  SD_J5_VEL,

  /// Joint phase current. (Unit: A)
  SD_J0_CUR,
  SD_J1_CUR,
  SD_J2_CUR,
  SD_J3_CUR,
  SD_J4_CUR,
  SD_J5_CUR,

  /// Variable representing the joint angles set in Begin. (Unit: degree)
  SD_BEGIN_J0,
  SD_BEGIN_J1,
  SD_BEGIN_J2,
  SD_BEGIN_J3,
  SD_BEGIN_J4,
  SD_BEGIN_J5,

  /// Joint motor controller temperature. (Unit: celsius)
  SD_TEMPERATURE_MC0,
  SD_TEMPERATURE_MC1,
  SD_TEMPERATURE_MC2,
  SD_TEMPERATURE_MC3,
  SD_TEMPERATURE_MC4,
  SD_TEMPERATURE_MC5,

  /// TCP values (position and orientation) with respect to the base (global) coordinate.
  SD_TCP_X,
  SD_TCP_Y,
  SD_TCP_Z,
  SD_TCP_RX,
  SD_TCP_RY,
  SD_TCP_RZ,

  /// Representing the default speed bar. The UI speed control bar value is displayed between 0 and 1.
  SD_DEFAULT_SPEED,

  /// Indicates whether the robot motion command is being executed.
  /// @note 1 = Idle
  /// @note 3 = Moving
  SD_ROBOT_STATE,

  /// Power information of the control box.
  /// @note SD_POWER_STATE >> 0 & 0x01 : 48V SMPS State
  /// @note SD_POWER_STATE >> 1 & 0x01 : Power switching information
  /// @note SD_POWER_STATE >> 2 & 0x01 : 24V SMPS State
  /// @note SD_POWER_STATE >> 3 & 0x01 : 48V switching information
  /// @note SD_POWER_STATE >> 4 & 0x01 : User (PC) power selection information
  /// @note SD_POWER_STATE >> 5 & 0x01 : Estop switch state
  SD_POWER_STATE,

  /// Whether the external collision detection function is on/off
  /// @note 0 = Collision detection mode off
  /// @note 1 = Collision detection mode on
  SD_COLLISION_DETECT_STATE,

  /// Whether to use direct teaching
  /// @note 0 = Free-drive (Direct teaching) off
  /// @note 1 = Free-drive (Direct teaching) on
  SD_IS_FREE_DRIVE_MODE,

  /// Indicates the robot's operation mode.
  /// @note 0 = Real mode
  /// @note 1 = Simulation mode
  SD_PG_MODE,

  /// This is a system variable representing the activation phase information of the robot
  SD_INIT_STATE_INFO,

  /// This is a system variable that indicates the activation error information
  SD_INIT_ERR,

  /// Variable with analog value (0~10V) of two analog input ports of tool flange board (TFB)
  SD_TFB_ANALOG_IN_0,
  SD_TFB_ANALOG_IN_1,

  /// A variable with a digital value (0 or 1) of the two digital input ports on the tool flange board (TFB)
  SD_TFB_DIGITAL_IN_0,
  SD_TFB_DIGITAL_IN_1,

  /// A variable with an output value (0 or 1) of the two digital output ports of the tool flange board (TFB)
  SD_TFB_DIGITAL_OUT_0,
  SD_TFB_DIGITAL_OUT_1,

  /// It is a variable indicating the voltage output information (0 or 12 or 24V) of the tool flange board (TFB)
  SD_TFB_VOLTAGE_OUT,

  /// A variable indicating whether an external collision detected.
  /// @note 0 = Idle
  /// @note 1 = External collision detected
  SD_OP_STAT_COLLISION_OCCUR,

  /// A variable indicating if a control box power problem / robot joint controller / other problem has occurred.
  /// @note 0 = Idle
  SD_OP_STAT_SOS_FLAG,

  /// A variable indicating just before self-collision during robot motion.
  /// @note 0 = Idle
  /// @note 1 = Entering self-collision range
  SD_OP_STAT_SELF_COLLISION,

  /// This is a variable indicating whether the program/robot is in the paused state.
  /// @note 0 = Idle
  /// @note 1 = Pause state
  SD_OP_STAT_ESTOP_OCCUR,

  /// This variable tells the user whether or not a singularity is present.
  /// @note 0 = Idle
  SD_OP_STAT_EMS_FLAG,

  /// Shows the information of the two protective stop terminals. (Din 16/17)
  SD_DIGITAL_IN_CONFIG_0,
  SD_DIGITAL_IN_CONFIG_1,

  /// This is a variable that checks whether a specific part of the robot has entered a specific area (Inbox).
  SD_INBOX_TRAP_FLAG_0,
  SD_INBOX_TRAP_FLAG_1,

  /// Inbox Check mode
  /// @note 0: No checking
  /// @note 1: Check Tool Flange Center (check whether the TFC is in the Inbox area)
  /// @note 2: Check Tool Center Point (check whether the TCP is in the Inbox area)
  /// @note 3: Check Tool Box (check whether the virtual box set at the end of the robot arm has entered in Inbox area.)
  /// @note 4: Check all (1, 2, 3)
  SD_INBOX_CHECK_MODE_0,
  SD_INBOX_CHECK_MODE_1,

  /// This is a variable indicating whether the socket of the corresponding number was normally opened and whether
  /// it was normally connected to the server.
  /// @note 1: creation and connection were performed normally.
  SD_SOCK_IS_OPEN_0,
  SD_SOCK_IS_OPEN_1,
  SD_SOCK_IS_OPEN_2,
  SD_SOCK_IS_OPEN_3,
  SD_SOCK_IS_OPEN_4,

  /// A variable indicating whether the read function was performed normally with the socket of the corresponding number.
  /// @note 1: the read was performed normally.
  SD_SOCK_LAST_READ_0,
  SD_SOCK_LAST_READ_1,
  SD_SOCK_LAST_READ_2,
  SD_SOCK_LAST_READ_3,
  SD_SOCK_LAST_READ_4,

  /// This is a variable indicating whether or not an act of tapping (tok tok) from outside the robot has occurred.
  SD_HAND_TOKTOK,

  /// Stores whether the motion has ended in a way that the robot's FinishAt (motion escape) condition.
  /// @note 0: the motion is finished by reaching the motion target point.
  /// @note 1: the FinishAt condition is satisfied and the operation is finished.
  SD_FINISH_AT_EVENT,

  /// TCP reference velocity
  SD_TCP_VEL_REF,

  /// Is is a variable that stores the time of unit movement.
  SD_MOTION_TIME,

  /// A variable representing the power the robot is using.
  SD_ARM_POWER,

  /// This is a variable that indicates whether the TPU (Teaching Pendant Unit, Tablet PC) is connected.
  SD_IS_TPU_CONNECT,

  /// Indicates whether the current program operation is executed in the Make page.
  SD_IS_RUN_IN_MAKE,

  /// Indicates whether the current program operation is executed in the Play page.
  SD_IS_RUN_IN_PLAY,

  /// Indicates whether the program termination is an intentional termination or an emergency termination.
  SD_IS_INTENDED_STOP,

  /// In a continuous motion such as MovePB / ITPL, it tells which point the robot is passing through.
  SD_MOVE_INDEX,

  SD_MOVE_INDEX_F,
  SD_MOVE_PROPERTY,
  SD_CURRENT_DELTA,
  SD_FORCE_TRAVEL_DIS,
  SD_EMG_BUTTON_STATE,
  SD_IS_IN_MAIN,
  SD_IS_HOME,
  SD_IS_BEGIN,
  SD_ID_NUMBER,
  SD_TF_LRF_DISTANCE,
  SD_TF_LRF_QUALITY,
  SD_BIT_0_3,
  SD_BIT_4_7,
  SD_BIT_8_11,
  SD_BIT_12_15,
  SD_BIT_0_7,
  SD_BIT_0_11,
  SD_BIT_0_15,
  OR_2FG7_STATE,
  OR_2FG7_POS_EXT,
  OR_2FG7_POS_INT,
  OR_2FG7_MIN_EXT,
  OR_2FG7_MAX_EXT,
  OR_2FG7_MIN_INT,
  OR_2FG7_MAX_INT,
  OR_RG_BUSY,
  OR_RG_GRIP,
  OR_RG_S1_PUSHED,
  OR_RG_S1_TRIGGERED,
  OR_RG_S2_PUSHED,
  OR_RG_S2_TRIGGERED,
  OR_RG_ERROR,
  OR_RG_WIDTH,
  OR_3FG_MIN_D,
  OR_3FG_MAX_D,
  OR_3FG_RAW_D,
  OR_3FG_REAL_D,
  OR_3FG_FORCE,
  OR_3FG_BUSY,
  OR_3FG_GRIP,
  OR_3FG_FORCE_GRIP,
  OR_3FG_CALIB,
  OR_SD_STATUS,
  OR_SD_WARNING,
  OR_SD_RPM,
  OR_SD_RPM_DEV,
  OR_SD_VIBRATION,
  OR_SD_ERROR,
  OR_SD_RPM_TAR,
  OR_SD_MOTOR_STOPPED,
  OR_SD_MOTOR_RUNNING,
  OR_SD_RAMP_UP,
  OR_SD_RAMP_DW,
  OR_SD_BUTTON,
  OR_VG_A,
  OR_VG_B,
  OR_VGP20_A,
  OR_VGP20_B,
  OR_VGP20_C,
  OR_VGP20_D,
  OR_VGP20_A_GRIP,
  OR_VGP20_A_RELEASE,
  OR_VGP20_B_GRIP,
  OR_VGP20_B_RELEASE,
  OR_VGP20_C_GRIP,
  OR_VGP20_C_RELEASE,
  OR_VGP20_D_GRIP,
  OR_VGP20_D_RELEASE,
  OR_VGP20_BUSY,
  OR_VGP20_ERROR,
  OR_MG10_STATUS,
  OR_MG10_ERROR,
  OR_MG10_STRENGTH,
  OR_SG_WIDTH,
  OR_SG_MAX_WIDTH,
  OR_SG_MIN_WIDTH,
  OR_SG_STATUS,
  OR_EYE_POS,
  OR_EYE_ERROR,
  OR_EYE_COUNT,
  OR_EYE_INSPECT_RESULT,
  OR_EYE_INSPECT_MATCH,
  OR_EYE_X,
  OR_EYE_Y,
  OR_EYE_Z,
  OR_EYE_RX,
  OR_EYE_RY,
  OR_EYE_RZ,
  JRT_ENCODER,
  JRT_JEGB,
  JRT_JEGG,
  PICKIT_POS,
  RC_PICKIT_NO_COMMAND,
  RC_PICKIT_CHECK_MODE,
  RC_PICKIT_CAPTURE_IMAGE,
  RC_PICKIT_PROCESS_IMAGE,
  RC_PICKIT_LOOK_FOR_OBJECTS,
  RC_PICKIT_LOOK_FOR_OBJECTS_WITH_RETRIES,
  RC_PICKIT_NEXT_OBJECT,
  RC_PICKIT_GET_PICK_POINT_DATA,
  RC_PICKIT_CONFIGURE,
  RC_PICKIT_SET_CYLINDER_DIM,
  RC_SAVE_ACTIVE_PRODUCT,
  RC_SAVE_ACTIVE_SETUP,
  RC_SAVE_BUILD_BACKGROUND,
  RC_PICKIT_FIND_CALIB_PLATE,
  RC_PICKIT_SAVE_SCENE,
  PICKIT_STATUS,
  PICKIT_VERSION,
  PICKIT_ROBOTTYPE,
  PICKIT_ROBOT_MODE,
  PICKIT_IDLE_MODE,
  PICKIT_OBJECT_FOUND,
  PICKIT_NO_OBJECTS,
  PICKIT_IMAGE_CAPTURED,
  PICKIT_NO_IMAGE_CAPTURED,
  PICKIT_EMPTY_ROI,
  PICKIT_GET_PICK_POINT_DATA_OK,
  PICKIT_GET_PICK_POINT_DATA_FAILED,
  PICKIT_CONFIG_OK,
  PICKIT_CONFIG_FAILED,
  PICKIT_BUILD_BKG_CLOUD_OK,
  PICKIT_BUILD_BKG_CLOUD_FAILED,
  PICKIT_FIND_CALIB_PLATE_OK,
  PICKIT_FIND_CALIB_PLATE_FAILED,
  PICKIT_SAVE_SNAPSHOT_OK,
  PICKIT_SAVE_SNAPSHOT_FAILED,
  PICKIT_UNKNOWN_COMMAND,
  PICKIT_TYPE_SQUARE,
  PICKIT_TYPE_RECTANGLE,
  PICKIT_TYPE_CIRCLE,
  PICKIT_TYPE_ELLIPSE,
  PICKIT_TYPE_CYLINDER,
  PICKIT_TYPE_SPHERE,
  PICKIT_TYPE_POINTCLOUD,
  PICKIT_TYPE_BLOB,
  PICKIT_X,
  PICKIT_Y,
  PICKIT_Z,
  PICKIT_RX,
  PICKIT_RY,
  PICKIT_RZ,
  PICKIT_P0,
  PICKIT_P1,
  PICKIT_P2,
  PICKIT_P3,
  PICKIT_P4,
  PICKIT_P5,
  PICKIT_P0F,
  PICKIT_P1F,
  PICKIT_P2F,
  PICKIT_P3F,
  PICKIT_P4F,
  PICKIT_P5F,
  PICKIT_M0,
  PICKIT_M1,
  ICE_INFO_CONNECTED,
  ICE_INFO_REQUESTING,
  ICE_INFO_USING,
  ICE_INFO_VECSIZE,
  ICE_INFO_MODE_COMM,
  ICE_INFO_MODE_CUP,
  ICE_INFO_TIME_ICE,
  ICE_INFO_TIME_WATER,
  ICE_INFO_AMBI_LOW,
  ICE_INFO_AMBI_HIGH,
  ICE_INFO_TMEP_AMBI,
  ICE_INFO_TMEP_EVAPO,
  ICE_INFO_TMEP_CONDEN,
  ICE_STATE_LAST_ICE_NO,
  ICE_STATE_LAST_ICE_YES,
  ICE_STATE_COMP_WORK,
  ICE_STATE_MOTOR_WORK,
  ICE_STATE_OUT_SOL,
  ICE_STATE_CUP_LEVEL,
  ICE_STATE_COMM_MODE,
  ICE_STATE_FULL_ICE,
  ICE_STATE_ERR_1,
  ICE_STATE_ERR_2,
  ICE_STATE_ERR_3,
  ICE_STATE_ERR_4,
  ICE_STATE_ERR_CODE,
  ICE_STATE_RD,
  SETECH_RDY,
  SETECH_ALM,
  SETECH_BUSY,
  SETECH_COMP,
  SETECH_OK,
  SETECH_NG_TRQH,
  SETECH_NG_TRQL,
  SETECH_NG_ANGH,
  SETECH_NG_ANGL,
  SETECH_NG_TIME,
  SETECH_NG_MONI,
  SETECH_NG_CH1,
  SETECH_NG_CH2,
  SETECH_NG_CH4,
  SETECH_NG_CH8,
  SETECH_NG_CH16,
  SD_NO_ARC_STATE,
  SD_DWELD_ARC,
  SD_DWELD_TOUCH,
  SD_DWELD_A,
  SD_DWELD_V,
  SD_DWELD_F,
  SD_DWELD_SET_A,
  SD_CONV_POS_TICK,
  SD_CONV_VEL_TICK,
};
```

### Operation mode

```c++
enum class OperationMode { Real, Simulation };
```

### Reference frame

```c++
enum class ReferenceFrame { 
  Base = 0,  ///< Base (Global) coordinate
  Tool,      ///< Tool (Local) coordinate
  User0,     ///< User coordinate 0
  User1,     ///< User coordinate 1
  User2      ///< User coordinate 2
};
```

### Blending option

```c++
enum class BlendingOption { 
  Ratio = 0,  ///< Blend based on Ratio
  Distance    ///< Blend based on Distance
};
```

### MovePB option

```c++
enum class MovePBOption { 
  Intended = 0,  ///< Intended (Follows the rotation value taught by the user)
  Constant,      ///< Constant (Keep the rotation value of the starting position)
  Smooth = 3     ///< Smooth (Similar to Intended, but with a smooth rate of rotation change)
};
```

### MoveITPL option

```c++
enum class MoveITPLOption {
  Intended = 0,    ///< Intended (Follows the rotation value taught by the user)
  Constant,        ///< Constant (Keep the rotation value of the starting position)
  Smooth = 3,      ///< Smooth (Similar to Intended, but with a smooth rate of rotation change)
  CAIntended = 5,  ///< CA-Intended (CA mode Intended)
  CAConstant,      ///< CA-Constant (CA mode Constant)
  CASmooth = 8     ///< CA-Smooth (CA mode Smooth)
};
```

### MoveLC property

```c++
enum class MoveLCProperty { 
  LinearMotion = 0, ///< Pass through linear motion
  CircularMotion    ///< Pass through circular motion
};
```

### MoveLC option

```c++
enum class MoveLCOption { 
  Intended = 0,  ///< Intended (Follows the rotation value taught by the user)
  Constant,      ///< Constant (Keep the rotation value of the starting position)
  Smooth = 3     ///< Smooth (Similar to Intended, but with a smooth rate of rotation change)
};
```

### MoveLB option

```c++
enum class MoveLBOption { 
  Intended = 0,   ///< Intended (Follows the rotation value taught by the user)
  Constant        ///< Constant (Keep the rotation value of the starting position)
};
```

### MoveC orientation option

```c++
enum class MoveCOrientationOption { 
  Intended = 0,   ///< Intended (Follows the rotation value taught by the user)
  Constant,       ///< Constant (Keep the rotation value of the starting position)
  Radial,         ///< Radial (Rotate the TCP according to the rotation of the circle)
  Smooth          ///< Smooth (Similar to Intended, but with a smooth rate of rotation change)
};
```

### MoveC rotation option

```c++
enum class MoveCRotationOption { 
  Intended = 0,   ///< Intended (rotate the same way as the Constant below.)
  Constant,       ///< Constant (Keep the rotation value of the starting position)
  Radial          ///< Radial (Rotate the TCP according to the rotation of the circle)
};
```

### MoveServoT option

```c++
enum class MoveServoTOption { None = 0, GravityCompensation = 1, FrictionCompensation = 2 };
```

### Collision mode

```c++
enum class CollisionMode {
  GeneralStop = 0,  ///< General Stop
  EvasionStop       ///< Evasion Stop
};
```

### Collision reaction mode

```c++
enum class CollisionReactionMode {
  PauseProgram = 0,  ///< Pause the program
  StopProgram        ///< Halt/stop the program flow
};
```

### Digital IO mode

```c++
enum class DigitalIOMode { Bypass = -1, Low = 0, High = 1 };
```

### Endian

```c++
enum class Endian { LittleEndian = 0, BigEndian };
```

## Struct types

### System State

```c++
typedef union {
  struct Data {
    /**
     * Header of this data structure
     *
     * @note header[0] = 0x24;
     * @note header[1] = size & 0xFF;
     * @note header[2] = (size >> 8) & 0xFF
     * @note header[3] = 0x03; // Type of this data
     */
    char header[4];

    /**
     * Basic timer. (Unit: sec)
     */
    float time;

    /**
     * Reference (desired) joint position. (Unit: deg)
     *
     * @note 0 = Base / 1 = Shoulder / 2 = Elbow / 3 = Wrist1 / 4 = Wrist2 / 5 = Wrist3
     */
    float jnt_ref[6];

    /**
     * Measured joint position. (Unit: deg)
     *
     * @note 0 = Base / 1 = Shoulder / 2 = Elbow / 3 = Wrist1 / 4 = Wrist2 / 5 = Wrist3
     * @note These values do not change in simulation mode.
     */
    float jnt_ang[6];

    /**
     * Measured joint current. (Unit: Amp)
     *
     * @note 0 = Base / 1 = Shoulder / 2 = Elbow / 3 = Wrist1 / 4 = Wrist2 / 5 = Wrist3
     * @note You can get joint torque by multiplying torque constant.
     */
    float jnt_cur[6];

    /**
     * TCP posture info based on reference-joint-angles (unit: mm & degree)
     *
     * @note 0 = X / 1 = Y / 2 = Z / 3 = Rx / 4 = Ry / 5 = Rz
     */
    float tcp_ref[6];

    /**
     * TCP posture info based on encoder-joint-angles (unit: mm & degree)
     *
     * @note 0 = X / 1 = Y / 2 = Z / 3 = Rx / 4 = Ry / 5 = Rz
     * @warning It is being transmitted overwritten based on the current reference.
     */
    float tcp_pos[6];

    /**
     * Control box analog input measurement information of each channel (unit: Voltage)
     *
     * @note Channel number: 0~3
     */
    float analog_in[4];

    /**
     * Control box analog output information of each channel (unit: Voltage)
     *
     * @note Channel number: 0~3

     */
    float analog_out[4];

    /**
     * Control box digital input measurement information of each channel (value: 0 or 1)
     *
     * @note Channel number: 0~15
     */
    int digital_in[16];

    /**
     * Control box digital output information of each channel (value: 0 or 1)
     *
     * @note Channel number: 0~15
     */
    int digital_out[16];

    /**
     * Measured temperature of each joint. (unit: celsius)
     *
     * @note 0 = Base / 1 = Shoulder / 2 = Elbow / 3 = Wrist1 / 4 = Wrist2 / 5 = Wrist3
     */
    float jnt_temperature[6];

    /**
     * Target program counter position during [STEP] function.
     */
    int task_pc;

    /**
     * Target program execution number in [PLAY] page.
     */
    int task_repeat;

    /**
     * Running program counter position.
     */
    int task_run_id;

    /**
     * Current program execution number in [PLAY] page.
     */
    int task_run_num;

    /**
     * Time since the program started (unit: second)
     */
    int task_run_time;

    /**
     * Basic state of 'Program Execution'
     *
     * @note 1 = Program not run / Idle
     * @note 3 = Program is running
     * @note 2 = Program is running + but 'Paused'state
     */
    int task_state;

    /**
     * Default speed multiplier value of robot motion (=speed bar in UI) (value: 0 ~ 1)
     */
    float default_speed;

    /**
     * Move (motion) state
     *
     * @note 1 = No motion command / Idle
     * @note 3 = Executing motion command(s)
     * @note 5 = No motion (Move) command + but executing Conveyor or Force control mode
     * @note 60+index = Under MovePB/ITPL/Pro command / index is passing waypoint number
     */
    int robot_state;

    /**
     * Information chunk to deliver various state information (power and others). It consists of a combination of bits.
     *
     * @note (information_chunk_1 >> 0) & 0b01 = Control Box's 48V input state
     * @note (information_chunk_1 >> 1) & 0b01 = Control Box's 48V output state
     * @note (information_chunk_1 >> 2) & 0b01 = Control Box's 24V input state
     * @note (information_chunk_1 >> 3) & 0b01 = Control Box's E-Stop state 1
     * @note (information_chunk_1 >> 4) & 0b01 = Control Box's User Switch state
     * @note (information_chunk_1 >> 5) & 0b01 = Control Box's E-Stop state 2
     * @note (information_chunk_1 >> 6) & 0b01 = Whether power is applied to the robot arm
     * @note (information_chunk_1 >> 7) & 0b01 = TFB's Direct teaching button is pressed
     * @note (information_chunk_1 >> 30) & 0b01 = Program Load state
     * @note (Whenever the Program load process is successful, 1 and 0 are continuously converted.)
     * @note (information_chunk_1 >> 31) & 0b01 = Program Transmit state (via TCP/IP Tablet UI, not for user)
     */
    int information_chunk_1;

    /**
     * Reserved / Not used
     */
    float reserved_1[6];

    /**
     * Basic state of each joint.
     *
     * @note 0 = Base / 1 = Shoulder / 2 = Elbow / 3 = Wrist1 / 4 = Wrist2 / 5 = Wrist3
     * @note Each int (4byte) consists of a combination of bits.
     * @note (jnt_info[#] >> 0) & 0b01 = Joint #'s FET state
     * @note (jnt_info[#] >> 1) & 0b01 = Joint #'s RUN state
     * @note (jnt_info[#] >> 2) & 0b01 = Joint #'s INIT state
     * @note (jnt_info[#] >> 3) & 0b01 = Joint #'s MODE state
     * @note (jnt_info[#] >> 4) & 0b01 = Joint #'s encoder state (Nonius err)
     * @note (jnt_info[#] >> 5) & 0b01 = Joint #'s encoder state (LowBatt err)
     * @note (jnt_info[#] >> 6) & 0b01 = Joint #'s encoder state (Calibration mode)
     * @note (jnt_info[#] >> 7) & 0b01 = Joint #'s encoder state (Multi-turn err)
     * @note (jnt_info[#] >> 8) & 0b01 = Joint #'s Error state (JAM err)
     * @note (jnt_info[#] >> 9) & 0b01 = Joint #'s Error state (CUR err)
     * @note (jnt_info[#] >> 10) & 0b01 = Joint #'s Error state (BIG err)
     * @note (jnt_info[#] >> 11) & 0b01 = Joint #'s Error state (INP err)
     * @note (jnt_info[#] >> 12) & 0b01 = Joint #'s Error state (FLT err)
     * @note (jnt_info[#] >> 13) & 0b01 = Joint #'s Error state (TMP err)
     * @note (jnt_info[#] >> 14) & 0b01 = Joint #'s Error state (PS1 err)
     * @note (jnt_info[#] >> 15) & 0b01 = Joint #'s Error state (PS2 err)
     */
    int jnt_info[6];

    /**
     * Out collision detection On/Off State (1=On / 0 = Off)
     */
    int collision_detect_onoff;

    /**
     * Free-drive (Gravity-compensation) On/Off State (1=On / 0 = Off)
     */
    int is_freedrive_mode;

    /**
     * Mode of operation: Simulation mode=1 / Real Robot mode=0
     */
    int real_vs_simulation_mode;

    /**
     * Robot arm activation (Initialization) stage info (0 -> 6)
     *
     * @note 0: default
     * @note 1: Power check
     * @note 2: Device check
     * @note 3: Servo Initialization check
     * @note 4: Parameter check
     * @note 5: Payload check
     * @note 6: Activation done
     */
    int init_state_info;

    /**
     * Error code during the arm activation (return value for UI)
     *
     * @warning Not for user
     */
    int init_error;

    /**
     * Robot-Tool-Flange analog input measurement information of each channel (unit: Voltage)
     *
     * @note Channel number: 0~1
     */
    float tfb_analog_in[2];

    /**
     * Robot-Tool-Flange digital input measurement information of each channel (value: 0 or 1)
     *
     * @note Channel number: 0~1
     */
    int tfb_digital_in[2];

    /**
     * Robot-Tool-Flange digital output information of each channel (value: 0 or 1)
     *
     * @note Channel number: 0~1
     */
    int tfb_digital_out[2];

    /**
     * Robot-Tool-Flage output voltage level (unit: Voltage)
     */
    float tfb_voltage_out;

    /**
     * Whether out-collision is detected (0 or 1)
     */
    int op_stat_collision_occur;

    /**
     * Robot Arm device error code during operation.
     *
     * @note 0 = None / 1 = Encoder err (PVL) / 2 = CPU err / 3 = Big err / 4 = Input err /
     * @note 5 = JAM err / 6 = Over current err / 7 = Position bound err / 8 = Mode err / 9 = Match err /
     * @note 10 = Over current/Low voltage err / 11 = Temperature err / 12 = Speed over err
     */
    int op_stat_sos_flag;

    /**
     * Whether self-collision is detected (0 or 1)
     */
    int op_stat_self_collision;

    /**
     * Pause state flag (0 or 1)
     */
    int op_stat_soft_estop_occur;

    /**
     * Software (kinematics) emergency stop situation
     *
     * @note 0 = None / 1 = Arm Stretch / 2= Cartesian Limit / 3=Joint Limit / 4=Un-solvable
     */
    int op_stat_ems_flag;

    /**
     * Information chunk to deliver various state information. It consists of a combination of bits.
     *
     * @note (information_chunk_2 >> 0) & 0b11 = Config digital input 16 (0 or 1) (Not for user)
     * @note (information_chunk_2 >> 2) & 0b1111111111111111 = Target welding voltage * 100
     */
    int information_chunk_2;

    /**
     * Information chunk to deliver various state information. It consists of a combination of bits.
     *
     * @note (information_chunk_3 >> 0) & 0b11 = Config digital input 17 (0 or 1) (Not for user)
     */
    int information_chunk_3;

    /**
     * Whether or not detected by the Inbox # check-function.
     *
     * @note # = In Box number: 0 or 1
     */
    int inbox_trap_flag[2];

    /**
     * Check-function mode of Inbox #.
     *
     * @note # = In Box number: 0 or 1
     * @note 0 = None / 1 = Check Tool Flange center / 2 = Check TCP / 3 = Check Tool Box / 4 = Check all
     */
    int inbox_check_mode[2];

    /**
     * External F/T force sensor value. Fx (Unit: N)
     */
    float eft_fx;

    /**
     * External F/T force sensor value. Fy (Unit: N)
     */
    float eft_fy;

    /**
     * External F/T force sensor value. Fz (Unit: N)
     */
    float eft_fz;

    /**
     * External F/T torque sensor value. Mx (Unit: Nm)
     */
    float eft_mx;

    /**
     * External F/T torque sensor value. My (Unit: Nm)
     */
    float eft_my;

    /**
     * External F/T torque sensor value. Mz (Unit: Nm)
     */
    float eft_mz;

    /**
     * Information chunk to deliver various state information. It consists of a combination of bits.
     *
     * @note (information_chunk_4 >> 0) & 0b11 = No-Arc Function On/Off (0 or 1)
     * @note (information_chunk_4 >> 2) & 0b111111 = Selected Tool List number
     * @note (information_chunk_4 >> 8) & 0b11 = External Joint (External axis) Jog On/Off (0 or 1)
     * @note (information_chunk_4 >> 10) & 0b01 = Tool Flange Digital Input 2
     * @note (information_chunk_4 >> 11) & 0b01 = Tool Flange Digital Input 3
     * @note (information_chunk_4 >> 12) & 0b01 = Tool Flange Digital Input 4
     * @note (information_chunk_4 >> 13) & 0b01 = Tool Flange Digital Input 5
     * @note (information_chunk_4 >> 14) & 0b01 = Arc Light On state (Not for user)
     * @note (information_chunk_4 >> 15) & 0b1111111111111 = Target welding current * 10
     * @note (information_chunk_4 >> 28) & 0b11 = Target welding voltage option (0 or 1)
     */
    int information_chunk_4;

    /**
     * Extended I/O board analog input measurement information of each channel (unit: Voltage)
     *
     * @note Channel number: 0~3
     */
    float extend_io1_analog_in[4];

    /**
     * Extended I/O board analog output information of each channel (unit: Voltage)
     *
     * @note Channel number: 0~3
     */
    float extend_io1_analog_out[4];

    /**
     * Extended I/O board digital input/output information. It consists of a combination of bits.
     *
     * @note (extend_io1_digital_info >> 0) & 0b01 = Extend I/O digital input # 0
     * @note (extend_io1_digital_info >> 1) & 0b01 = Extend I/O digital input # 1
     * @note ..
     * @note (extend_io1_digital_info >> 15) & 0b01 = Extend I/O digital input # 15
     * @note (extend_io1_digital_info >> 16) & 0b01 = Extend I/O digital output # 0
     * @note (extend_io1_digital_info >> 17) & 0b01 = Extend I/O digital output # 1
     * @note ..
     * @note (extend_io1_digital_info >> 31) & 0b01 = Extend I/O digital input # 15
     */
    unsigned int extend_io1_digital_info;

    /**
     * Reference angle of each external-joint (auxiliary joint). (unit: degree)
     *
     * @note External joint number: 0~5
     */
    float aa_joint_ref[6];

    /**
     * Data chunk about the control box safety board
     */
    unsigned int safety_board_stat_info;
  } sdata;

  float fdata[kMaxSharedData];
  int idata[kMaxSharedData];
} SystemState;
```