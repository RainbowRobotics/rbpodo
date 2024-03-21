/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#pragma once

#include <array>
#include <cstring>
#include <random>
#include <sstream>
#include <utility>

namespace rb::podo {
static inline const size_t kMaxSharedData{128};

inline std::string randstr(int length = 5) {
  static std::random_device seed;
  static std::default_random_engine random_engine(seed());
  static std::uniform_int_distribution<int> uniform_distribution(0, 10 + 26 - 1);

  std::stringstream ss;
  ss << "#";
  for (int i = 0; i < length; i++) {
    int r = uniform_distribution(random_engine);
    if (r < 10) {
      ss << (char)(r + '0');
    } else {
      ss << (char)(r - 10 + 'a');
    }
  }
  ss << "_";
  return ss.str();
}

class StandardVector {
 public:
  using Joint = std::array<double, 6>;
  using Point = std::array<double, 6>;
  using JointRef = Joint&;
  using PointRef = Point&;
  using JointConstRef = const Joint&;
  using PointConstRef = const Point&;

  static inline std::string joint_to_string(JointConstRef joint) {
    std::stringstream ss;
    ss << "jnt[";
    for (int i = 0; i < 6; i++) {
      if (i != 0) {
        ss << ", ";
      }
      ss << joint[i];
    }
    ss << "]";
    return ss.str();
  }

  static inline std::string point_to_string(PointConstRef point) {
    std::stringstream ss;
    ss << "pnt[";
    for (int i = 0; i < 6; i++) {
      if (i != 0) {
        ss << ", ";
      }
      ss << point[i];
    }
    ss << "]";
    return ss.str();
  }
};

#ifdef EIGEN_VERSION_AT_LEAST
#if EIGEN_VERSION_AT_LEAST(3, 3, 7)
class EigenVector {
 public:
  using Joint = Eigen::Vector<double, 6>;
  using Point = Eigen::Vector<double, 6>;
  using JointRef = Eigen::Ref<Joint>;
  using PointRef = Eigen::Ref<Point>;
  using JointConstRef = const Eigen::Ref<const Joint>&;
  using PointConstRef = const Eigen::Ref<const Point>&;

  static std::string joint_to_string(JointConstRef joint) {
    std::stringstream ss;
    ss << "jnt" << joint.format({Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"});
    return ss.str();
  }

  static std::string point_to_string(PointConstRef point) {
    std::stringstream ss;
    ss << "pnt" << point.format({Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"});
    return ss.str();
  }
};
#endif
#endif

struct ControlBoxInfo {
  int system_version{};
  int robot_box_type{};

  [[nodiscard]] std::string str() const {
    std::stringstream ss;
    ss << "{ \"SystemVersion\": " << system_version << ", \"RobotBoxType\": " << robot_box_type << " }";
    return ss.str();
  }
};

enum class RobotState {
  Unknown,
  Idle,
  Moving,
};

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

enum class OperationMode { Real, Simulation };

enum class ReferenceFrame { 
  Base = 0,  ///< Base (Global) coordinate
  Tool,      ///< Tool (Local) coordinate
  User0,     ///< User coordinate 0
  User1,     ///< User coordinate 1
  User2      ///< User coordinate 2
};

enum class BlendingOption { 
  Ratio = 0,  ///< Blend based on Ratio
  Distance    ///< Blend based on Distance
};

enum class MovePBOption { 
  Intended = 0,  ///< Intended (Follows the rotation value taught by the user)
  Constant,      ///< Constant (Keep the rotation value of the starting position)
  Smooth = 3     ///< Smooth (Similar to Intended, but with a smooth rate of rotation change)
};

enum class MoveITPLOption {
  Intended = 0,    ///< Intended (Follows the rotation value taught by the user)
  Constant,        ///< Constant (Keep the rotation value of the starting position)
  Smooth = 3,      ///< Smooth (Similar to Intended, but with a smooth rate of rotation change)
  CAIntended = 5,  ///< CA-Intended (CA mode Intended)
  CAConstant,      ///< CA-Constant (CA mode Constant)
  CASmooth = 8     ///< CA-Smooth (CA mode Smooth)
};

enum class MoveLCProperty { 
  LinearMotion = 0, ///< Pass through linear motion
  CircularMotion    ///< Pass through circular motion
};

enum class MoveLCOption { 
  Intended = 0,  ///< Intended (Follows the rotation value taught by the user)
  Constant,      ///< Constant (Keep the rotation value of the starting position)
  Smooth = 3     ///< Smooth (Similar to Intended, but with a smooth rate of rotation change)
};

enum class MoveLBOption { 
  Intended = 0,   ///< Intended (Follows the rotation value taught by the user)
  Constant        ///< Constant (Keep the rotation value of the starting position)
};

enum class MoveCOrientationOption { 
  Intended = 0,   ///< Intended (Follows the rotation value taught by the user)
  Constant,       ///< Constant (Keep the rotation value of the starting position)
  Radial,         ///< Radial (Rotate the TCP according to the rotation of the circle)
  Smooth          ///< Smooth (Similar to Intended, but with a smooth rate of rotation change)
};

enum class MoveCRotationOption { 
  Intended = 0,   ///< Intended (rotate the same way as the Constant below.)
  Constant,       ///< Constant (Keep the rotation value of the starting position)
  Radial          ///< Radial (Rotate the TCP according to the rotation of the circle)
};

enum class MoveServoTOption { NoCompensation = 0, GravityCompensation = 1, FrictionCompensation = 2 };

enum class CollisionMode {
  GeneralStop = 0,  ///< General Stop
  EvasionStop       ///< Evasion Stop
};

enum class CollisionReactionMode {
  PauseProgram = 0,  ///< Pause the program
  StopProgram        ///< Halt/stop the program flow
};

/// Mode of the digital output in control box.
enum class DigitalIOMode {
  Bypass = -1,  ///< Bypass
  Low = 0,      ///< Low
  High = 1      ///< High
};

enum class Endian { LittleEndian = 0, BigEndian };

/// Gripper Product List
enum class GripperModel {
  Robotiq_Hand_E = 0,
  Robotiq_2F_85,
  Robotiq_2F1_40,
  Robotiq_E_Pick,
  Robotis_RH_P12_RN,
  JRT_JEGB_4285,
  JRT_JEGB_42140,
  DH_AG_95,
  Setech_NutRunner,
  OnRobot2FG7,
  JRT_JEGG,
  RB_5FDG,
  OnRobot_RG2,
  OnRobot_RG6,
  OnRobot_VG10,
  OnRobot_VGC10,
  OnRobot_3FG15,
  OnRobot_Sander,
  OnRobot_VGP20,
  OnRobot_MG10,
  OnRobot_SoftGripper,
  JRT_JEGC,
  Schunk_Coact,
  Baumer_Laser_Sensor,
  Mighty_12LF,
  JRT_JEGH_3520,
};

enum class GripperConnectionPoint { ToolFlange = 0, ControlBox, ToolFlange_Advanced };

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

inline std::string to_string(SystemVariable var) {
  switch (var) {
    case SystemVariable::SD_TIME:
      return "SD_TIME";
    case SystemVariable::SD_TIMER_0:
      return "SD_TIMER_0";
    case SystemVariable::SD_TIMER_1:
      return "SD_TIMER_1";
    case SystemVariable::SD_TIMER_2:
      return "SD_TIMER_2";
    case SystemVariable::SD_TIMER_3:
      return "SD_TIMER_3";
    case SystemVariable::SD_TIMER_4:
      return "SD_TIMER_4";
    case SystemVariable::SD_TIMER_5:
      return "SD_TIMER_5";
    case SystemVariable::SD_TIMER_6:
      return "SD_TIMER_6";
    case SystemVariable::SD_TIMER_7:
      return "SD_TIMER_7";
    case SystemVariable::SD_TIMER_8:
      return "SD_TIMER_8";
    case SystemVariable::SD_TIMER_9:
      return "SD_TIMER_9";
    case SystemVariable::SD_J0_REF:
      return "SD_J0_REF";
    case SystemVariable::SD_J1_REF:
      return "SD_J1_REF";
    case SystemVariable::SD_J2_REF:
      return "SD_J2_REF";
    case SystemVariable::SD_J3_REF:
      return "SD_J3_REF";
    case SystemVariable::SD_J4_REF:
      return "SD_J4_REF";
    case SystemVariable::SD_J5_REF:
      return "SD_J5_REF";
    case SystemVariable::SD_J0_ANG:
      return "SD_J0_ANG";
    case SystemVariable::SD_J1_ANG:
      return "SD_J1_ANG";
    case SystemVariable::SD_J2_ANG:
      return "SD_J2_ANG";
    case SystemVariable::SD_J3_ANG:
      return "SD_J3_ANG";
    case SystemVariable::SD_J4_ANG:
      return "SD_J4_ANG";
    case SystemVariable::SD_J5_ANG:
      return "SD_J5_ANG";
    case SystemVariable::SD_J0_VEL:
      return "SD_J0_VEL";
    case SystemVariable::SD_J1_VEL:
      return "SD_J1_VEL";
    case SystemVariable::SD_J2_VEL:
      return "SD_J2_VEL";
    case SystemVariable::SD_J3_VEL:
      return "SD_J3_VEL";
    case SystemVariable::SD_J4_VEL:
      return "SD_J4_VEL";
    case SystemVariable::SD_J5_VEL:
      return "SD_J5_VEL";
    case SystemVariable::SD_J0_CUR:
      return "SD_J0_CUR";
    case SystemVariable::SD_J1_CUR:
      return "SD_J1_CUR";
    case SystemVariable::SD_J2_CUR:
      return "SD_J2_CUR";
    case SystemVariable::SD_J3_CUR:
      return "SD_J3_CUR";
    case SystemVariable::SD_J4_CUR:
      return "SD_J4_CUR";
    case SystemVariable::SD_J5_CUR:
      return "SD_J5_CUR";
    case SystemVariable::SD_BEGIN_J0:
      return "SD_BEGIN_J0";
    case SystemVariable::SD_BEGIN_J1:
      return "SD_BEGIN_J1";
    case SystemVariable::SD_BEGIN_J2:
      return "SD_BEGIN_J2";
    case SystemVariable::SD_BEGIN_J3:
      return "SD_BEGIN_J3";
    case SystemVariable::SD_BEGIN_J4:
      return "SD_BEGIN_J4";
    case SystemVariable::SD_BEGIN_J5:
      return "SD_BEGIN_J5";
    case SystemVariable::SD_TEMPERATURE_MC0:
      return "SD_TEMPERATURE_MC0";
    case SystemVariable::SD_TEMPERATURE_MC1:
      return "SD_TEMPERATURE_MC1";
    case SystemVariable::SD_TEMPERATURE_MC2:
      return "SD_TEMPERATURE_MC2";
    case SystemVariable::SD_TEMPERATURE_MC3:
      return "SD_TEMPERATURE_MC3";
    case SystemVariable::SD_TEMPERATURE_MC4:
      return "SD_TEMPERATURE_MC4";
    case SystemVariable::SD_TEMPERATURE_MC5:
      return "SD_TEMPERATURE_MC5";
    case SystemVariable::SD_TCP_X:
      return "SD_TCP_X";
    case SystemVariable::SD_TCP_Y:
      return "SD_TCP_Y";
    case SystemVariable::SD_TCP_Z:
      return "SD_TCP_Z";
    case SystemVariable::SD_TCP_RX:
      return "SD_TCP_RX";
    case SystemVariable::SD_TCP_RY:
      return "SD_TCP_RY";
    case SystemVariable::SD_TCP_RZ:
      return "SD_TCP_RZ";
    case SystemVariable::SD_DEFAULT_SPEED:
      return "SD_DEFAULT_SPEED";
    case SystemVariable::SD_ROBOT_STATE:
      return "SD_ROBOT_STATE";
    case SystemVariable::SD_POWER_STATE:
      return "SD_POWER_STATE";
    case SystemVariable::SD_COLLISION_DETECT_STATE:
      return "SD_COLLISION_DETECT_STATE";
    case SystemVariable::SD_IS_FREE_DRIVE_MODE:
      return "SD_IS_FREE_DRIVE_MODE";
    case SystemVariable::SD_PG_MODE:
      return "SD_PG_MODE";
    case SystemVariable::SD_INIT_STATE_INFO:
      return "SD_INIT_STATE_INFO";
    case SystemVariable::SD_INIT_ERR:
      return "SD_INIT_ERR";
    case SystemVariable::SD_TFB_ANALOG_IN_0:
      return "SD_TFB_ANALOG_IN_0";
    case SystemVariable::SD_TFB_ANALOG_IN_1:
      return "SD_TFB_ANALOG_IN_1";
    case SystemVariable::SD_TFB_DIGITAL_IN_0:
      return "SD_TFB_DIGITAL_IN_0";
    case SystemVariable::SD_TFB_DIGITAL_IN_1:
      return "SD_TFB_DIGITAL_IN_1";
    case SystemVariable::SD_TFB_DIGITAL_OUT_0:
      return "SD_TFB_DIGITAL_OUT_0";
    case SystemVariable::SD_TFB_DIGITAL_OUT_1:
      return "SD_TFB_DIGITAL_OUT_1";
    case SystemVariable::SD_TFB_VOLTAGE_OUT:
      return "SD_TFB_VOLTAGE_OUT";
    case SystemVariable::SD_OP_STAT_COLLISION_OCCUR:
      return "SD_OP_STAT_COLLISION_OCCUR";
    case SystemVariable::SD_OP_STAT_SOS_FLAG:
      return "SD_OP_STAT_SOS_FLAG";
    case SystemVariable::SD_OP_STAT_SELF_COLLISION:
      return "SD_OP_STAT_SELF_COLLISION";
    case SystemVariable::SD_OP_STAT_ESTOP_OCCUR:
      return "SD_OP_STAT_ESTOP_OCCUR";
    case SystemVariable::SD_OP_STAT_EMS_FLAG:
      return "SD_OP_STAT_EMS_FLAG";
    case SystemVariable::SD_DIGITAL_IN_CONFIG_0:
      return "SD_DIGITAL_IN_CONFIG_0";
    case SystemVariable::SD_DIGITAL_IN_CONFIG_1:
      return "SD_DIGITAL_IN_CONFIG_1";
    case SystemVariable::SD_INBOX_TRAP_FLAG_0:
      return "SD_INBOX_TRAP_FLAG_0";
    case SystemVariable::SD_INBOX_TRAP_FLAG_1:
      return "SD_INBOX_TRAP_FLAG_1";
    case SystemVariable::SD_INBOX_CHECK_MODE_0:
      return "SD_INBOX_CHECK_MODE_0";
    case SystemVariable::SD_INBOX_CHECK_MODE_1:
      return "SD_INBOX_CHECK_MODE_1";
    case SystemVariable::SD_SOCK_IS_OPEN_0:
      return "SD_SOCK_IS_OPEN_0";
    case SystemVariable::SD_SOCK_IS_OPEN_1:
      return "SD_SOCK_IS_OPEN_1";
    case SystemVariable::SD_SOCK_IS_OPEN_2:
      return "SD_SOCK_IS_OPEN_2";
    case SystemVariable::SD_SOCK_IS_OPEN_3:
      return "SD_SOCK_IS_OPEN_3";
    case SystemVariable::SD_SOCK_IS_OPEN_4:
      return "SD_SOCK_IS_OPEN_4";
    case SystemVariable::SD_SOCK_LAST_READ_0:
      return "SD_SOCK_LAST_READ_0";
    case SystemVariable::SD_SOCK_LAST_READ_1:
      return "SD_SOCK_LAST_READ_1";
    case SystemVariable::SD_SOCK_LAST_READ_2:
      return "SD_SOCK_LAST_READ_2";
    case SystemVariable::SD_SOCK_LAST_READ_3:
      return "SD_SOCK_LAST_READ_3";
    case SystemVariable::SD_SOCK_LAST_READ_4:
      return "SD_SOCK_LAST_READ_4";
    case SystemVariable::SD_HAND_TOKTOK:
      return "SD_HAND_TOKTOK";
    case SystemVariable::SD_FINISH_AT_EVENT:
      return "SD_FINISH_AT_EVENT";
    case SystemVariable::SD_TCP_VEL_REF:
      return "SD_TCP_VEL_REF";
    case SystemVariable::SD_MOTION_TIME:
      return "SD_MOTION_TIME";
    case SystemVariable::SD_ARM_POWER:
      return "SD_ARM_POWER";
    case SystemVariable::SD_IS_TPU_CONNECT:
      return "SD_IS_TPU_CONNECT";
    case SystemVariable::SD_IS_RUN_IN_MAKE:
      return "SD_IS_RUN_IN_MAKE";
    case SystemVariable::SD_IS_RUN_IN_PLAY:
      return "SD_IS_RUN_IN_PLAY";
    case SystemVariable::SD_IS_INTENDED_STOP:
      return "SD_IS_INTENDED_STOP";
    case SystemVariable::SD_MOVE_INDEX:
      return "SD_MOVE_INDEX";
    case SystemVariable::SD_MOVE_INDEX_F:
      return "SD_MOVE_INDEX_F";
    case SystemVariable::SD_MOVE_PROPERTY:
      return "SD_MOVE_PROPERTY";
    case SystemVariable::SD_CURRENT_DELTA:
      return "SD_CURRENT_DELTA";
    case SystemVariable::SD_FORCE_TRAVEL_DIS:
      return "SD_FORCE_TRAVEL_DIS";
    case SystemVariable::SD_EMG_BUTTON_STATE:
      return "SD_EMG_BUTTON_STATE";
    case SystemVariable::SD_IS_IN_MAIN:
      return "SD_IS_IN_MAIN";
    case SystemVariable::SD_IS_HOME:
      return "SD_IS_HOME";
    case SystemVariable::SD_IS_BEGIN:
      return "SD_IS_BEGIN";
    case SystemVariable::SD_ID_NUMBER:
      return "SD_ID_NUMBER";
    case SystemVariable::SD_TF_LRF_DISTANCE:
      return "SD_TF_LRF_DISTANCE";
    case SystemVariable::SD_TF_LRF_QUALITY:
      return "SD_TF_LRF_QUALITY";
    case SystemVariable::SD_BIT_0_3:
      return "SD_BIT_0_3";
    case SystemVariable::SD_BIT_4_7:
      return "SD_BIT_4_7";
    case SystemVariable::SD_BIT_8_11:
      return "SD_BIT_8_11";
    case SystemVariable::SD_BIT_12_15:
      return "SD_BIT_12_15";
    case SystemVariable::SD_BIT_0_7:
      return "SD_BIT_0_7";
    case SystemVariable::SD_BIT_0_11:
      return "SD_BIT_0_11";
    case SystemVariable::SD_BIT_0_15:
      return "SD_BIT_0_15";
    case SystemVariable::OR_2FG7_STATE:
      return "OR_2FG7_STATE";
    case SystemVariable::OR_2FG7_POS_EXT:
      return "OR_2FG7_POS_EXT";
    case SystemVariable::OR_2FG7_POS_INT:
      return "OR_2FG7_POS_INT";
    case SystemVariable::OR_2FG7_MIN_EXT:
      return "OR_2FG7_MIN_EXT";
    case SystemVariable::OR_2FG7_MAX_EXT:
      return "OR_2FG7_MAX_EXT";
    case SystemVariable::OR_2FG7_MIN_INT:
      return "OR_2FG7_MIN_INT";
    case SystemVariable::OR_2FG7_MAX_INT:
      return "OR_2FG7_MAX_INT";
    case SystemVariable::OR_RG_BUSY:
      return "OR_RG_BUSY";
    case SystemVariable::OR_RG_GRIP:
      return "OR_RG_GRIP";
    case SystemVariable::OR_RG_S1_PUSHED:
      return "OR_RG_S1_PUSHED";
    case SystemVariable::OR_RG_S1_TRIGGERED:
      return "OR_RG_S1_TRIGGERED";
    case SystemVariable::OR_RG_S2_PUSHED:
      return "OR_RG_S2_PUSHED";
    case SystemVariable::OR_RG_S2_TRIGGERED:
      return "OR_RG_S2_TRIGGERED";
    case SystemVariable::OR_RG_ERROR:
      return "OR_RG_ERROR";
    case SystemVariable::OR_RG_WIDTH:
      return "OR_RG_WIDTH";
    case SystemVariable::OR_3FG_MIN_D:
      return "OR_3FG_MIN_D";
    case SystemVariable::OR_3FG_MAX_D:
      return "OR_3FG_MAX_D";
    case SystemVariable::OR_3FG_RAW_D:
      return "OR_3FG_RAW_D";
    case SystemVariable::OR_3FG_REAL_D:
      return "OR_3FG_REAL_D";
    case SystemVariable::OR_3FG_FORCE:
      return "OR_3FG_FORCE";
    case SystemVariable::OR_3FG_BUSY:
      return "OR_3FG_BUSY";
    case SystemVariable::OR_3FG_GRIP:
      return "OR_3FG_GRIP";
    case SystemVariable::OR_3FG_FORCE_GRIP:
      return "OR_3FG_FORCE_GRIP";
    case SystemVariable::OR_3FG_CALIB:
      return "OR_3FG_CALIB";
    case SystemVariable::OR_SD_STATUS:
      return "OR_SD_STATUS";
    case SystemVariable::OR_SD_WARNING:
      return "OR_SD_WARNING";
    case SystemVariable::OR_SD_RPM:
      return "OR_SD_RPM";
    case SystemVariable::OR_SD_RPM_DEV:
      return "OR_SD_RPM_DEV";
    case SystemVariable::OR_SD_VIBRATION:
      return "OR_SD_VIBRATION";
    case SystemVariable::OR_SD_ERROR:
      return "OR_SD_ERROR";
    case SystemVariable::OR_SD_RPM_TAR:
      return "OR_SD_RPM_TAR";
    case SystemVariable::OR_SD_MOTOR_STOPPED:
      return "OR_SD_MOTOR_STOPPED";
    case SystemVariable::OR_SD_MOTOR_RUNNING:
      return "OR_SD_MOTOR_RUNNING";
    case SystemVariable::OR_SD_RAMP_UP:
      return "OR_SD_RAMP_UP";
    case SystemVariable::OR_SD_RAMP_DW:
      return "OR_SD_RAMP_DW";
    case SystemVariable::OR_SD_BUTTON:
      return "OR_SD_BUTTON";
    case SystemVariable::OR_VG_A:
      return "OR_VG_A";
    case SystemVariable::OR_VG_B:
      return "OR_VG_B";
    case SystemVariable::OR_VGP20_A:
      return "OR_VGP20_A";
    case SystemVariable::OR_VGP20_B:
      return "OR_VGP20_B";
    case SystemVariable::OR_VGP20_C:
      return "OR_VGP20_C";
    case SystemVariable::OR_VGP20_D:
      return "OR_VGP20_D";
    case SystemVariable::OR_VGP20_A_GRIP:
      return "OR_VGP20_A_GRIP";
    case SystemVariable::OR_VGP20_A_RELEASE:
      return "OR_VGP20_A_RELEASE";
    case SystemVariable::OR_VGP20_B_GRIP:
      return "OR_VGP20_B_GRIP";
    case SystemVariable::OR_VGP20_B_RELEASE:
      return "OR_VGP20_B_RELEASE";
    case SystemVariable::OR_VGP20_C_GRIP:
      return "OR_VGP20_C_GRIP";
    case SystemVariable::OR_VGP20_C_RELEASE:
      return "OR_VGP20_C_RELEASE";
    case SystemVariable::OR_VGP20_D_GRIP:
      return "OR_VGP20_D_GRIP";
    case SystemVariable::OR_VGP20_D_RELEASE:
      return "OR_VGP20_D_RELEASE";
    case SystemVariable::OR_VGP20_BUSY:
      return "OR_VGP20_BUSY";
    case SystemVariable::OR_VGP20_ERROR:
      return "OR_VGP20_ERROR";
    case SystemVariable::OR_MG10_STATUS:
      return "OR_MG10_STATUS";
    case SystemVariable::OR_MG10_ERROR:
      return "OR_MG10_ERROR";
    case SystemVariable::OR_MG10_STRENGTH:
      return "OR_MG10_STRENGTH";
    case SystemVariable::OR_SG_WIDTH:
      return "OR_SG_WIDTH";
    case SystemVariable::OR_SG_MAX_WIDTH:
      return "OR_SG_MAX_WIDTH";
    case SystemVariable::OR_SG_MIN_WIDTH:
      return "OR_SG_MIN_WIDTH";
    case SystemVariable::OR_SG_STATUS:
      return "OR_SG_STATUS";
    case SystemVariable::OR_EYE_POS:
      return "OR_EYE_POS";
    case SystemVariable::OR_EYE_ERROR:
      return "OR_EYE_ERROR";
    case SystemVariable::OR_EYE_COUNT:
      return "OR_EYE_COUNT";
    case SystemVariable::OR_EYE_INSPECT_RESULT:
      return "OR_EYE_INSPECT_RESULT";
    case SystemVariable::OR_EYE_INSPECT_MATCH:
      return "OR_EYE_INSPECT_MATCH";
    case SystemVariable::OR_EYE_X:
      return "OR_EYE_X";
    case SystemVariable::OR_EYE_Y:
      return "OR_EYE_Y";
    case SystemVariable::OR_EYE_Z:
      return "OR_EYE_Z";
    case SystemVariable::OR_EYE_RX:
      return "OR_EYE_RX";
    case SystemVariable::OR_EYE_RY:
      return "OR_EYE_RY";
    case SystemVariable::OR_EYE_RZ:
      return "OR_EYE_RZ";
    case SystemVariable::JRT_ENCODER:
      return "JRT_ENCODER";
    case SystemVariable::JRT_JEGB:
      return "JRT_JEGB";
    case SystemVariable::JRT_JEGG:
      return "JRT_JEGG";
    case SystemVariable::PICKIT_POS:
      return "PICKIT_POS";
    case SystemVariable::RC_PICKIT_NO_COMMAND:
      return "RC_PICKIT_NO_COMMAND";
    case SystemVariable::RC_PICKIT_CHECK_MODE:
      return "RC_PICKIT_CHECK_MODE";
    case SystemVariable::RC_PICKIT_CAPTURE_IMAGE:
      return "RC_PICKIT_CAPTURE_IMAGE";
    case SystemVariable::RC_PICKIT_PROCESS_IMAGE:
      return "RC_PICKIT_PROCESS_IMAGE";
    case SystemVariable::RC_PICKIT_LOOK_FOR_OBJECTS:
      return "RC_PICKIT_LOOK_FOR_OBJECTS";
    case SystemVariable::RC_PICKIT_LOOK_FOR_OBJECTS_WITH_RETRIES:
      return "RC_PICKIT_LOOK_FOR_OBJECTS_WITH_RETRIES";
    case SystemVariable::RC_PICKIT_NEXT_OBJECT:
      return "RC_PICKIT_NEXT_OBJECT";
    case SystemVariable::RC_PICKIT_GET_PICK_POINT_DATA:
      return "RC_PICKIT_GET_PICK_POINT_DATA";
    case SystemVariable::RC_PICKIT_CONFIGURE:
      return "RC_PICKIT_CONFIGURE";
    case SystemVariable::RC_PICKIT_SET_CYLINDER_DIM:
      return "RC_PICKIT_SET_CYLINDER_DIM";
    case SystemVariable::RC_SAVE_ACTIVE_PRODUCT:
      return "RC_SAVE_ACTIVE_PRODUCT";
    case SystemVariable::RC_SAVE_ACTIVE_SETUP:
      return "RC_SAVE_ACTIVE_SETUP";
    case SystemVariable::RC_SAVE_BUILD_BACKGROUND:
      return "RC_SAVE_BUILD_BACKGROUND";
    case SystemVariable::RC_PICKIT_FIND_CALIB_PLATE:
      return "RC_PICKIT_FIND_CALIB_PLATE";
    case SystemVariable::RC_PICKIT_SAVE_SCENE:
      return "RC_PICKIT_SAVE_SCENE";
    case SystemVariable::PICKIT_STATUS:
      return "PICKIT_STATUS";
    case SystemVariable::PICKIT_VERSION:
      return "PICKIT_VERSION";
    case SystemVariable::PICKIT_ROBOTTYPE:
      return "PICKIT_ROBOTTYPE";
    case SystemVariable::PICKIT_ROBOT_MODE:
      return "PICKIT_ROBOT_MODE";
    case SystemVariable::PICKIT_IDLE_MODE:
      return "PICKIT_IDLE_MODE";
    case SystemVariable::PICKIT_OBJECT_FOUND:
      return "PICKIT_OBJECT_FOUND";
    case SystemVariable::PICKIT_NO_OBJECTS:
      return "PICKIT_NO_OBJECTS";
    case SystemVariable::PICKIT_IMAGE_CAPTURED:
      return "PICKIT_IMAGE_CAPTURED";
    case SystemVariable::PICKIT_NO_IMAGE_CAPTURED:
      return "PICKIT_NO_IMAGE_CAPTURED";
    case SystemVariable::PICKIT_EMPTY_ROI:
      return "PICKIT_EMPTY_ROI";
    case SystemVariable::PICKIT_GET_PICK_POINT_DATA_OK:
      return "PICKIT_GET_PICK_POINT_DATA_OK";
    case SystemVariable::PICKIT_GET_PICK_POINT_DATA_FAILED:
      return "PICKIT_GET_PICK_POINT_DATA_FAILED";
    case SystemVariable::PICKIT_CONFIG_OK:
      return "PICKIT_CONFIG_OK";
    case SystemVariable::PICKIT_CONFIG_FAILED:
      return "PICKIT_CONFIG_FAILED";
    case SystemVariable::PICKIT_BUILD_BKG_CLOUD_OK:
      return "PICKIT_BUILD_BKG_CLOUD_OK";
    case SystemVariable::PICKIT_BUILD_BKG_CLOUD_FAILED:
      return "PICKIT_BUILD_BKG_CLOUD_FAILED";
    case SystemVariable::PICKIT_FIND_CALIB_PLATE_OK:
      return "PICKIT_FIND_CALIB_PLATE_OK";
    case SystemVariable::PICKIT_FIND_CALIB_PLATE_FAILED:
      return "PICKIT_FIND_CALIB_PLATE_FAILED";
    case SystemVariable::PICKIT_SAVE_SNAPSHOT_OK:
      return "PICKIT_SAVE_SNAPSHOT_OK";
    case SystemVariable::PICKIT_SAVE_SNAPSHOT_FAILED:
      return "PICKIT_SAVE_SNAPSHOT_FAILED";
    case SystemVariable::PICKIT_UNKNOWN_COMMAND:
      return "PICKIT_UNKNOWN_COMMAND";
    case SystemVariable::PICKIT_TYPE_SQUARE:
      return "PICKIT_TYPE_SQUARE";
    case SystemVariable::PICKIT_TYPE_RECTANGLE:
      return "PICKIT_TYPE_RECTANGLE";
    case SystemVariable::PICKIT_TYPE_CIRCLE:
      return "PICKIT_TYPE_CIRCLE";
    case SystemVariable::PICKIT_TYPE_ELLIPSE:
      return "PICKIT_TYPE_ELLIPSE";
    case SystemVariable::PICKIT_TYPE_CYLINDER:
      return "PICKIT_TYPE_CYLINDER";
    case SystemVariable::PICKIT_TYPE_SPHERE:
      return "PICKIT_TYPE_SPHERE";
    case SystemVariable::PICKIT_TYPE_POINTCLOUD:
      return "PICKIT_TYPE_POINTCLOUD";
    case SystemVariable::PICKIT_TYPE_BLOB:
      return "PICKIT_TYPE_BLOB";
    case SystemVariable::PICKIT_X:
      return "PICKIT_X";
    case SystemVariable::PICKIT_Y:
      return "PICKIT_Y";
    case SystemVariable::PICKIT_Z:
      return "PICKIT_Z";
    case SystemVariable::PICKIT_RX:
      return "PICKIT_RX";
    case SystemVariable::PICKIT_RY:
      return "PICKIT_RY";
    case SystemVariable::PICKIT_RZ:
      return "PICKIT_RZ";
    case SystemVariable::PICKIT_P0:
      return "PICKIT_P0";
    case SystemVariable::PICKIT_P1:
      return "PICKIT_P1";
    case SystemVariable::PICKIT_P2:
      return "PICKIT_P2";
    case SystemVariable::PICKIT_P3:
      return "PICKIT_P3";
    case SystemVariable::PICKIT_P4:
      return "PICKIT_P4";
    case SystemVariable::PICKIT_P5:
      return "PICKIT_P5";
    case SystemVariable::PICKIT_P0F:
      return "PICKIT_P0F";
    case SystemVariable::PICKIT_P1F:
      return "PICKIT_P1F";
    case SystemVariable::PICKIT_P2F:
      return "PICKIT_P2F";
    case SystemVariable::PICKIT_P3F:
      return "PICKIT_P3F";
    case SystemVariable::PICKIT_P4F:
      return "PICKIT_P4F";
    case SystemVariable::PICKIT_P5F:
      return "PICKIT_P5F";
    case SystemVariable::PICKIT_M0:
      return "PICKIT_M0";
    case SystemVariable::PICKIT_M1:
      return "PICKIT_M1";
    case SystemVariable::ICE_INFO_CONNECTED:
      return "ICE_INFO_CONNECTED";
    case SystemVariable::ICE_INFO_REQUESTING:
      return "ICE_INFO_REQUESTING";
    case SystemVariable::ICE_INFO_USING:
      return "ICE_INFO_USING";
    case SystemVariable::ICE_INFO_VECSIZE:
      return "ICE_INFO_VECSIZE";
    case SystemVariable::ICE_INFO_MODE_COMM:
      return "ICE_INFO_MODE_COMM";
    case SystemVariable::ICE_INFO_MODE_CUP:
      return "ICE_INFO_MODE_CUP";
    case SystemVariable::ICE_INFO_TIME_ICE:
      return "ICE_INFO_TIME_ICE";
    case SystemVariable::ICE_INFO_TIME_WATER:
      return "ICE_INFO_TIME_WATER";
    case SystemVariable::ICE_INFO_AMBI_LOW:
      return "ICE_INFO_AMBI_LOW";
    case SystemVariable::ICE_INFO_AMBI_HIGH:
      return "ICE_INFO_AMBI_HIGH";
    case SystemVariable::ICE_INFO_TMEP_AMBI:
      return "ICE_INFO_TMEP_AMBI";
    case SystemVariable::ICE_INFO_TMEP_EVAPO:
      return "ICE_INFO_TMEP_EVAPO";
    case SystemVariable::ICE_INFO_TMEP_CONDEN:
      return "ICE_INFO_TMEP_CONDEN";
    case SystemVariable::ICE_STATE_LAST_ICE_NO:
      return "ICE_STATE_LAST_ICE_NO";
    case SystemVariable::ICE_STATE_LAST_ICE_YES:
      return "ICE_STATE_LAST_ICE_YES";
    case SystemVariable::ICE_STATE_COMP_WORK:
      return "ICE_STATE_COMP_WORK";
    case SystemVariable::ICE_STATE_MOTOR_WORK:
      return "ICE_STATE_MOTOR_WORK";
    case SystemVariable::ICE_STATE_OUT_SOL:
      return "ICE_STATE_OUT_SOL";
    case SystemVariable::ICE_STATE_CUP_LEVEL:
      return "ICE_STATE_CUP_LEVEL";
    case SystemVariable::ICE_STATE_COMM_MODE:
      return "ICE_STATE_COMM_MODE";
    case SystemVariable::ICE_STATE_FULL_ICE:
      return "ICE_STATE_FULL_ICE";
    case SystemVariable::ICE_STATE_ERR_1:
      return "ICE_STATE_ERR_1";
    case SystemVariable::ICE_STATE_ERR_2:
      return "ICE_STATE_ERR_2";
    case SystemVariable::ICE_STATE_ERR_3:
      return "ICE_STATE_ERR_3";
    case SystemVariable::ICE_STATE_ERR_4:
      return "ICE_STATE_ERR_4";
    case SystemVariable::ICE_STATE_ERR_CODE:
      return "ICE_STATE_ERR_CODE";
    case SystemVariable::ICE_STATE_RD:
      return "ICE_STATE_RD";
    case SystemVariable::SETECH_RDY:
      return "SETECH_RDY";
    case SystemVariable::SETECH_ALM:
      return "SETECH_ALM";
    case SystemVariable::SETECH_BUSY:
      return "SETECH_BUSY";
    case SystemVariable::SETECH_COMP:
      return "SETECH_COMP";
    case SystemVariable::SETECH_OK:
      return "SETECH_OK";
    case SystemVariable::SETECH_NG_TRQH:
      return "SETECH_NG_TRQH";
    case SystemVariable::SETECH_NG_TRQL:
      return "SETECH_NG_TRQL";
    case SystemVariable::SETECH_NG_ANGH:
      return "SETECH_NG_ANGH";
    case SystemVariable::SETECH_NG_ANGL:
      return "SETECH_NG_ANGL";
    case SystemVariable::SETECH_NG_TIME:
      return "SETECH_NG_TIME";
    case SystemVariable::SETECH_NG_MONI:
      return "SETECH_NG_MONI";
    case SystemVariable::SETECH_NG_CH1:
      return "SETECH_NG_CH1";
    case SystemVariable::SETECH_NG_CH2:
      return "SETECH_NG_CH2";
    case SystemVariable::SETECH_NG_CH4:
      return "SETECH_NG_CH4";
    case SystemVariable::SETECH_NG_CH8:
      return "SETECH_NG_CH8";
    case SystemVariable::SETECH_NG_CH16:
      return "SETECH_NG_CH16";
    case SystemVariable::SD_NO_ARC_STATE:
      return "SD_NO_ARC_STATE";
    case SystemVariable::SD_DWELD_ARC:
      return "SD_DWELD_ARC";
    case SystemVariable::SD_DWELD_TOUCH:
      return "SD_DWELD_TOUCH";
    case SystemVariable::SD_DWELD_A:
      return "SD_DWELD_A";
    case SystemVariable::SD_DWELD_V:
      return "SD_DWELD_V";
    case SystemVariable::SD_DWELD_F:
      return "SD_DWELD_F";
    case SystemVariable::SD_DWELD_SET_A:
      return "SD_DWELD_SET_A";
    case SystemVariable::SD_CONV_POS_TICK:
      return "SD_CONV_POS_TICK";
    case SystemVariable::SD_CONV_VEL_TICK:
      return "SD_CONV_VEL_TICK";
  }
  return "";
}

}  // namespace rb::podo