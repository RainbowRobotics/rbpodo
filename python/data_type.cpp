/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#include <Eigen/Core>

#include "pybind11/eigen.h"
#include "pybind11/numpy.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "rbpodo/data_type.hpp"

namespace py = pybind11;
using namespace rb::podo;

void pybind11_data_type(py::module_& m) {
  py::class_<ControlBoxInfo>(m, "ControlBoxInfo")
      .def_readonly("system_version", &ControlBoxInfo::system_version)
      .def_readonly("robot_box_type", &ControlBoxInfo::robot_box_type)
      .def("__repr__", [](const ControlBoxInfo& self) { return self.str(); });

  py::enum_<RobotState>(m, "RobotState")
      .value("Unknown", RobotState::Unknown)
      .value("Idle", RobotState::Idle)
      .value("Moving", RobotState::Moving);

  py::enum_<OperationMode>(m, "OperationMode")
      .value("Real", OperationMode::Real)
      .value("Simulation", OperationMode::Simulation);

  py::enum_<ReferenceFrame>(m, "ReferenceFrame")
      .value("Base", ReferenceFrame::Base, "Base (Global) coordinate")
      .value("Tool", ReferenceFrame::Tool, "Tool (Local) coordinate")
      .value("User0", ReferenceFrame::User0, "User coordinate 0")
      .value("User1", ReferenceFrame::User1, "User coordinate 1")
      .value("User2", ReferenceFrame::User2, "User coordinate 2");

  py::enum_<BlendingOption>(m, "BlendingOption")
      .value("Ratio", BlendingOption::Ratio, "Blend based on Ratio")
      .value("Distance", BlendingOption::Distance, "Blend based on Distance");

  py::enum_<MovePBOption>(m, "MovePBOption")
      .value("Intended", MovePBOption::Intended, "Intended (Follows the rotation value taught by the user)")
      .value("Constant", MovePBOption::Constant, "Constant (Keep the rotation value of the starting position)")
      .value("Smooth", MovePBOption::Smooth, "Smooth (Similar to Intended, but with a smooth rate of rotation change)");

  py::enum_<MoveITPLOption>(m, "MoveITPLOption")
      .value("Intended", MoveITPLOption::Intended, "Intended (Follows the rotation value taught by the user)")
      .value("Constant", MoveITPLOption::Constant, "Constant (Keep the rotation value of the starting position)")
      .value("Smooth", MoveITPLOption::Smooth,
             "Smooth (Similar to Intended, but with a smooth rate of rotation change)")
      .value("CAIntended", MoveITPLOption::CAIntended, "CA-Intended (CA mode Intended)")
      .value("CAConstant", MoveITPLOption::CAConstant, "CA-Constant (CA mode Constant)")
      .value("CASmooth", MoveITPLOption::CASmooth, "CA-Smooth (CA mode Smooth)");

  py::enum_<MoveLCProperty>(m, "MoveLCProperty")
      .value("LinearMotion", MoveLCProperty::LinearMotion, "Pass through linear motion")
      .value("CircularMotion", MoveLCProperty::CircularMotion, "Pass through circular motion");

  py::enum_<MoveLCOption>(m, "MoveLCOption")
      .value("Intended", MoveLCOption::Intended, "Intended (Follows the rotation value taught by the user)")
      .value("Constant", MoveLCOption::Constant, "Constant (Keep the rotation value of the starting position)")
      .value("Smooth", MoveLCOption::Smooth, "Smooth (Similar to Intended, but with a smooth rate of rotation change)");

  py::enum_<MoveLBOption>(m, "MoveLBOption")
      .value("Intended", MoveLBOption::Intended, "Intended (Follows the rotation value taught by the user)")
      .value("Constant", MoveLBOption::Constant, "Constant (Keep the rotation value of the starting position)");

  py::enum_<MoveCOrientationOption>(m, "MoveCOrientationOption")
      .value("Intended", MoveCOrientationOption::Intended, "Intended (Follows the rotation value taught by the user)")
      .value("Constant", MoveCOrientationOption::Constant, "Constant (Keep the rotation value of the starting position)")
      .value("Radial", MoveCOrientationOption::Radial, "Radial (Rotate the TCP according to the rotation of the circle)")
      .value("Smooth", MoveCOrientationOption::Smooth, "Smooth (Similar to Intended, but with a smooth rate of rotation change)");

  py::enum_<MoveCRotationOption>(m, "MoveCRotationOption")
      .value("Intended", MoveCRotationOption::Intended, "Intended (rotate the same way as the Constant below.)")
      .value("Constant", MoveCRotationOption::Constant, "Constant (Keep the rotation value of the starting position)")
      .value("Radial", MoveCRotationOption::Radial, "Radial (Rotate the TCP according to the rotation of the circle)");

  py::enum_<MoveServoTOption>(m, "MoveServoTOption")
      .value("NoCompensation", MoveServoTOption::NoCompensation)
      .value("GravityCompensation", MoveServoTOption::GravityCompensation)
      .value("FrictionCompensation", MoveServoTOption::FrictionCompensation);

  py::enum_<CollisionMode>(m, "CollisionMode")
      .value("GeneralStop", CollisionMode::GeneralStop, "General Stop")
      .value("EvasionStop", CollisionMode::EvasionStop, "Evasion Stop");

  py::enum_<CollisionReactionMode>(m, "CollisionFollowUpMode")
      .value("PauseProgram", CollisionReactionMode::PauseProgram, "Pause the program")
      .value("StopProgram", CollisionReactionMode::StopProgram, "Halt/stop the program flow");

  py::enum_<DigitalIOMode>(m, "DigitalIOMode")
      .value("Bypass", DigitalIOMode::Bypass, "Bypass")
      .value("Low", DigitalIOMode::Low, "Low")
      .value("High", DigitalIOMode::High, "High");

  py::enum_<Endian>(m, "Endian").value("LittleEndian", Endian::LittleEndian).value("BigEndian", Endian::BigEndian);

  py::enum_<SystemVariable>(m, "SystemVariable")
      .value("SD_TIME", SystemVariable::SD_TIME)
      .value("SD_TIMER_0", SystemVariable::SD_TIMER_0)
      .value("SD_TIMER_1", SystemVariable::SD_TIMER_1)
      .value("SD_TIMER_2", SystemVariable::SD_TIMER_2)
      .value("SD_TIMER_3", SystemVariable::SD_TIMER_3)
      .value("SD_TIMER_4", SystemVariable::SD_TIMER_4)
      .value("SD_TIMER_5", SystemVariable::SD_TIMER_5)
      .value("SD_TIMER_6", SystemVariable::SD_TIMER_6)
      .value("SD_TIMER_7", SystemVariable::SD_TIMER_7)
      .value("SD_TIMER_8", SystemVariable::SD_TIMER_8)
      .value("SD_TIMER_9", SystemVariable::SD_TIMER_9)
      .value("SD_J0_REF", SystemVariable::SD_J0_REF)
      .value("SD_J1_REF", SystemVariable::SD_J1_REF)
      .value("SD_J2_REF", SystemVariable::SD_J2_REF)
      .value("SD_J3_REF", SystemVariable::SD_J3_REF)
      .value("SD_J4_REF", SystemVariable::SD_J4_REF)
      .value("SD_J5_REF", SystemVariable::SD_J5_REF)
      .value("SD_J0_ANG", SystemVariable::SD_J0_ANG)
      .value("SD_J1_ANG", SystemVariable::SD_J1_ANG)
      .value("SD_J2_ANG", SystemVariable::SD_J2_ANG)
      .value("SD_J3_ANG", SystemVariable::SD_J3_ANG)
      .value("SD_J4_ANG", SystemVariable::SD_J4_ANG)
      .value("SD_J5_ANG", SystemVariable::SD_J5_ANG)
      .value("SD_J0_VEL", SystemVariable::SD_J0_VEL)
      .value("SD_J1_VEL", SystemVariable::SD_J1_VEL)
      .value("SD_J2_VEL", SystemVariable::SD_J2_VEL)
      .value("SD_J3_VEL", SystemVariable::SD_J3_VEL)
      .value("SD_J4_VEL", SystemVariable::SD_J4_VEL)
      .value("SD_J5_VEL", SystemVariable::SD_J5_VEL)
      .value("SD_J0_CUR", SystemVariable::SD_J0_CUR)
      .value("SD_J1_CUR", SystemVariable::SD_J1_CUR)
      .value("SD_J2_CUR", SystemVariable::SD_J2_CUR)
      .value("SD_J3_CUR", SystemVariable::SD_J3_CUR)
      .value("SD_J4_CUR", SystemVariable::SD_J4_CUR)
      .value("SD_J5_CUR", SystemVariable::SD_J5_CUR)
      .value("SD_BEGIN_J0", SystemVariable::SD_BEGIN_J0)
      .value("SD_BEGIN_J1", SystemVariable::SD_BEGIN_J1)
      .value("SD_BEGIN_J2", SystemVariable::SD_BEGIN_J2)
      .value("SD_BEGIN_J3", SystemVariable::SD_BEGIN_J3)
      .value("SD_BEGIN_J4", SystemVariable::SD_BEGIN_J4)
      .value("SD_BEGIN_J5", SystemVariable::SD_BEGIN_J5)
      .value("SD_TEMPERATURE_MC0", SystemVariable::SD_TEMPERATURE_MC0)
      .value("SD_TEMPERATURE_MC1", SystemVariable::SD_TEMPERATURE_MC1)
      .value("SD_TEMPERATURE_MC2", SystemVariable::SD_TEMPERATURE_MC2)
      .value("SD_TEMPERATURE_MC3", SystemVariable::SD_TEMPERATURE_MC3)
      .value("SD_TEMPERATURE_MC4", SystemVariable::SD_TEMPERATURE_MC4)
      .value("SD_TEMPERATURE_MC5", SystemVariable::SD_TEMPERATURE_MC5)
      .value("SD_TCP_X", SystemVariable::SD_TCP_X)
      .value("SD_TCP_Y", SystemVariable::SD_TCP_Y)
      .value("SD_TCP_Z", SystemVariable::SD_TCP_Z)
      .value("SD_TCP_RX", SystemVariable::SD_TCP_RX)
      .value("SD_TCP_RY", SystemVariable::SD_TCP_RY)
      .value("SD_TCP_RZ", SystemVariable::SD_TCP_RZ)
      .value("SD_DEFAULT_SPEED", SystemVariable::SD_DEFAULT_SPEED)
      .value("SD_ROBOT_STATE", SystemVariable::SD_ROBOT_STATE)
      .value("SD_POWER_STATE", SystemVariable::SD_POWER_STATE)
      .value("SD_COLLISION_DETECT_STATE", SystemVariable::SD_COLLISION_DETECT_STATE)
      .value("SD_IS_FREE_DRIVE_MODE", SystemVariable::SD_IS_FREE_DRIVE_MODE)
      .value("SD_PG_MODE", SystemVariable::SD_PG_MODE)
      .value("SD_INIT_STATE_INFO", SystemVariable::SD_INIT_STATE_INFO)
      .value("SD_INIT_ERR", SystemVariable::SD_INIT_ERR)
      .value("SD_TFB_ANALOG_IN_0", SystemVariable::SD_TFB_ANALOG_IN_0)
      .value("SD_TFB_ANALOG_IN_1", SystemVariable::SD_TFB_ANALOG_IN_1)
      .value("SD_TFB_DIGITAL_IN_0", SystemVariable::SD_TFB_DIGITAL_IN_0)
      .value("SD_TFB_DIGITAL_IN_1", SystemVariable::SD_TFB_DIGITAL_IN_1)
      .value("SD_TFB_DIGITAL_OUT_0", SystemVariable::SD_TFB_DIGITAL_OUT_0)
      .value("SD_TFB_DIGITAL_OUT_1", SystemVariable::SD_TFB_DIGITAL_OUT_1)
      .value("SD_TFB_VOLTAGE_OUT", SystemVariable::SD_TFB_VOLTAGE_OUT)
      .value("SD_OP_STAT_COLLISION_OCCUR", SystemVariable::SD_OP_STAT_COLLISION_OCCUR)
      .value("SD_OP_STAT_SOS_FLAG", SystemVariable::SD_OP_STAT_SOS_FLAG)
      .value("SD_OP_STAT_SELF_COLLISION", SystemVariable::SD_OP_STAT_SELF_COLLISION)
      .value("SD_OP_STAT_ESTOP_OCCUR", SystemVariable::SD_OP_STAT_ESTOP_OCCUR)
      .value("SD_OP_STAT_EMS_FLAG", SystemVariable::SD_OP_STAT_EMS_FLAG)
      .value("SD_DIGITAL_IN_CONFIG_0", SystemVariable::SD_DIGITAL_IN_CONFIG_0)
      .value("SD_DIGITAL_IN_CONFIG_1", SystemVariable::SD_DIGITAL_IN_CONFIG_1)
      .value("SD_INBOX_TRAP_FLAG_0", SystemVariable::SD_INBOX_TRAP_FLAG_0)
      .value("SD_INBOX_TRAP_FLAG_1", SystemVariable::SD_INBOX_TRAP_FLAG_1)
      .value("SD_INBOX_CHECK_MODE_0", SystemVariable::SD_INBOX_CHECK_MODE_0)
      .value("SD_INBOX_CHECK_MODE_1", SystemVariable::SD_INBOX_CHECK_MODE_1)
      .value("SD_SOCK_IS_OPEN_0", SystemVariable::SD_SOCK_IS_OPEN_0)
      .value("SD_SOCK_IS_OPEN_1", SystemVariable::SD_SOCK_IS_OPEN_1)
      .value("SD_SOCK_IS_OPEN_2", SystemVariable::SD_SOCK_IS_OPEN_2)
      .value("SD_SOCK_IS_OPEN_3", SystemVariable::SD_SOCK_IS_OPEN_3)
      .value("SD_SOCK_IS_OPEN_4", SystemVariable::SD_SOCK_IS_OPEN_4)
      .value("SD_SOCK_LAST_READ_0", SystemVariable::SD_SOCK_LAST_READ_0)
      .value("SD_SOCK_LAST_READ_1", SystemVariable::SD_SOCK_LAST_READ_1)
      .value("SD_SOCK_LAST_READ_2", SystemVariable::SD_SOCK_LAST_READ_2)
      .value("SD_SOCK_LAST_READ_3", SystemVariable::SD_SOCK_LAST_READ_3)
      .value("SD_SOCK_LAST_READ_4", SystemVariable::SD_SOCK_LAST_READ_4)
      .value("SD_HAND_TOKTOK", SystemVariable::SD_HAND_TOKTOK)
      .value("SD_FINISH_AT_EVENT", SystemVariable::SD_FINISH_AT_EVENT)
      .value("SD_TCP_VEL_REF", SystemVariable::SD_TCP_VEL_REF)
      .value("SD_MOTION_TIME", SystemVariable::SD_MOTION_TIME)
      .value("SD_ARM_POWER", SystemVariable::SD_ARM_POWER)
      .value("SD_IS_TPU_CONNECT", SystemVariable::SD_IS_TPU_CONNECT)
      .value("SD_IS_RUN_IN_MAKE", SystemVariable::SD_IS_RUN_IN_MAKE)
      .value("SD_IS_RUN_IN_PLAY", SystemVariable::SD_IS_RUN_IN_PLAY)
      .value("SD_IS_INTENDED_STOP", SystemVariable::SD_IS_INTENDED_STOP)
      .value("SD_MOVE_INDEX", SystemVariable::SD_MOVE_INDEX)
      .value("SD_MOVE_INDEX_F", SystemVariable::SD_MOVE_INDEX_F)
      .value("SD_MOVE_PROPERTY", SystemVariable::SD_MOVE_PROPERTY)
      .value("SD_CURRENT_DELTA", SystemVariable::SD_CURRENT_DELTA)
      .value("SD_FORCE_TRAVEL_DIS", SystemVariable::SD_FORCE_TRAVEL_DIS)
      .value("SD_EMG_BUTTON_STATE", SystemVariable::SD_EMG_BUTTON_STATE)
      .value("SD_IS_IN_MAIN", SystemVariable::SD_IS_IN_MAIN)
      .value("SD_IS_HOME", SystemVariable::SD_IS_HOME)
      .value("SD_IS_BEGIN", SystemVariable::SD_IS_BEGIN)
      .value("SD_ID_NUMBER", SystemVariable::SD_ID_NUMBER)
      .value("SD_TF_LRF_DISTANCE", SystemVariable::SD_TF_LRF_DISTANCE)
      .value("SD_TF_LRF_QUALITY", SystemVariable::SD_TF_LRF_QUALITY)
      .value("SD_BIT_0_3", SystemVariable::SD_BIT_0_3)
      .value("SD_BIT_4_7", SystemVariable::SD_BIT_4_7)
      .value("SD_BIT_8_11", SystemVariable::SD_BIT_8_11)
      .value("SD_BIT_12_15", SystemVariable::SD_BIT_12_15)
      .value("SD_BIT_0_7", SystemVariable::SD_BIT_0_7)
      .value("SD_BIT_0_11", SystemVariable::SD_BIT_0_11)
      .value("SD_BIT_0_15", SystemVariable::SD_BIT_0_15)
      .value("OR_2FG7_STATE", SystemVariable::OR_2FG7_STATE)
      .value("OR_2FG7_POS_EXT", SystemVariable::OR_2FG7_POS_EXT)
      .value("OR_2FG7_POS_INT", SystemVariable::OR_2FG7_POS_INT)
      .value("OR_2FG7_MIN_EXT", SystemVariable::OR_2FG7_MIN_EXT)
      .value("OR_2FG7_MAX_EXT", SystemVariable::OR_2FG7_MAX_EXT)
      .value("OR_2FG7_MIN_INT", SystemVariable::OR_2FG7_MIN_INT)
      .value("OR_2FG7_MAX_INT", SystemVariable::OR_2FG7_MAX_INT)
      .value("OR_RG_BUSY", SystemVariable::OR_RG_BUSY)
      .value("OR_RG_GRIP", SystemVariable::OR_RG_GRIP)
      .value("OR_RG_S1_PUSHED", SystemVariable::OR_RG_S1_PUSHED)
      .value("OR_RG_S1_TRIGGERED", SystemVariable::OR_RG_S1_TRIGGERED)
      .value("OR_RG_S2_PUSHED", SystemVariable::OR_RG_S2_PUSHED)
      .value("OR_RG_S2_TRIGGERED", SystemVariable::OR_RG_S2_TRIGGERED)
      .value("OR_RG_ERROR", SystemVariable::OR_RG_ERROR)
      .value("OR_RG_WIDTH", SystemVariable::OR_RG_WIDTH)
      .value("OR_3FG_MIN_D", SystemVariable::OR_3FG_MIN_D)
      .value("OR_3FG_MAX_D", SystemVariable::OR_3FG_MAX_D)
      .value("OR_3FG_RAW_D", SystemVariable::OR_3FG_RAW_D)
      .value("OR_3FG_REAL_D", SystemVariable::OR_3FG_REAL_D)
      .value("OR_3FG_FORCE", SystemVariable::OR_3FG_FORCE)
      .value("OR_3FG_BUSY", SystemVariable::OR_3FG_BUSY)
      .value("OR_3FG_GRIP", SystemVariable::OR_3FG_GRIP)
      .value("OR_3FG_FORCE_GRIP", SystemVariable::OR_3FG_FORCE_GRIP)
      .value("OR_3FG_CALIB", SystemVariable::OR_3FG_CALIB)
      .value("OR_SD_STATUS", SystemVariable::OR_SD_STATUS)
      .value("OR_SD_WARNING", SystemVariable::OR_SD_WARNING)
      .value("OR_SD_RPM", SystemVariable::OR_SD_RPM)
      .value("OR_SD_RPM_DEV", SystemVariable::OR_SD_RPM_DEV)
      .value("OR_SD_VIBRATION", SystemVariable::OR_SD_VIBRATION)
      .value("OR_SD_ERROR", SystemVariable::OR_SD_ERROR)
      .value("OR_SD_RPM_TAR", SystemVariable::OR_SD_RPM_TAR)
      .value("OR_SD_MOTOR_STOPPED", SystemVariable::OR_SD_MOTOR_STOPPED)
      .value("OR_SD_MOTOR_RUNNING", SystemVariable::OR_SD_MOTOR_RUNNING)
      .value("OR_SD_RAMP_UP", SystemVariable::OR_SD_RAMP_UP)
      .value("OR_SD_RAMP_DW", SystemVariable::OR_SD_RAMP_DW)
      .value("OR_SD_BUTTON", SystemVariable::OR_SD_BUTTON)
      .value("OR_VG_A", SystemVariable::OR_VG_A)
      .value("OR_VG_B", SystemVariable::OR_VG_B)
      .value("OR_VGP20_A", SystemVariable::OR_VGP20_A)
      .value("OR_VGP20_B", SystemVariable::OR_VGP20_B)
      .value("OR_VGP20_C", SystemVariable::OR_VGP20_C)
      .value("OR_VGP20_D", SystemVariable::OR_VGP20_D)
      .value("OR_VGP20_A_GRIP", SystemVariable::OR_VGP20_A_GRIP)
      .value("OR_VGP20_A_RELEASE", SystemVariable::OR_VGP20_A_RELEASE)
      .value("OR_VGP20_B_GRIP", SystemVariable::OR_VGP20_B_GRIP)
      .value("OR_VGP20_B_RELEASE", SystemVariable::OR_VGP20_B_RELEASE)
      .value("OR_VGP20_C_GRIP", SystemVariable::OR_VGP20_C_GRIP)
      .value("OR_VGP20_C_RELEASE", SystemVariable::OR_VGP20_C_RELEASE)
      .value("OR_VGP20_D_GRIP", SystemVariable::OR_VGP20_D_GRIP)
      .value("OR_VGP20_D_RELEASE", SystemVariable::OR_VGP20_D_RELEASE)
      .value("OR_VGP20_BUSY", SystemVariable::OR_VGP20_BUSY)
      .value("OR_VGP20_ERROR", SystemVariable::OR_VGP20_ERROR)
      .value("OR_MG10_STATUS", SystemVariable::OR_MG10_STATUS)
      .value("OR_MG10_ERROR", SystemVariable::OR_MG10_ERROR)
      .value("OR_MG10_STRENGTH", SystemVariable::OR_MG10_STRENGTH)
      .value("OR_SG_WIDTH", SystemVariable::OR_SG_WIDTH)
      .value("OR_SG_MAX_WIDTH", SystemVariable::OR_SG_MAX_WIDTH)
      .value("OR_SG_MIN_WIDTH", SystemVariable::OR_SG_MIN_WIDTH)
      .value("OR_SG_STATUS", SystemVariable::OR_SG_STATUS)
      .value("OR_EYE_POS", SystemVariable::OR_EYE_POS)
      .value("OR_EYE_ERROR", SystemVariable::OR_EYE_ERROR)
      .value("OR_EYE_COUNT", SystemVariable::OR_EYE_COUNT)
      .value("OR_EYE_INSPECT_RESULT", SystemVariable::OR_EYE_INSPECT_RESULT)
      .value("OR_EYE_INSPECT_MATCH", SystemVariable::OR_EYE_INSPECT_MATCH)
      .value("OR_EYE_X", SystemVariable::OR_EYE_X)
      .value("OR_EYE_Y", SystemVariable::OR_EYE_Y)
      .value("OR_EYE_Z", SystemVariable::OR_EYE_Z)
      .value("OR_EYE_RX", SystemVariable::OR_EYE_RX)
      .value("OR_EYE_RY", SystemVariable::OR_EYE_RY)
      .value("OR_EYE_RZ", SystemVariable::OR_EYE_RZ)
      .value("JRT_ENCODER", SystemVariable::JRT_ENCODER)
      .value("JRT_JEGB", SystemVariable::JRT_JEGB)
      .value("JRT_JEGG", SystemVariable::JRT_JEGG)
      .value("PICKIT_POS", SystemVariable::PICKIT_POS)
      .value("RC_PICKIT_NO_COMMAND", SystemVariable::RC_PICKIT_NO_COMMAND)
      .value("RC_PICKIT_CHECK_MODE", SystemVariable::RC_PICKIT_CHECK_MODE)
      .value("RC_PICKIT_CAPTURE_IMAGE", SystemVariable::RC_PICKIT_CAPTURE_IMAGE)
      .value("RC_PICKIT_PROCESS_IMAGE", SystemVariable::RC_PICKIT_PROCESS_IMAGE)
      .value("RC_PICKIT_LOOK_FOR_OBJECTS", SystemVariable::RC_PICKIT_LOOK_FOR_OBJECTS)
      .value("RC_PICKIT_LOOK_FOR_OBJECTS_WITH_RETRIES", SystemVariable::RC_PICKIT_LOOK_FOR_OBJECTS_WITH_RETRIES)
      .value("RC_PICKIT_NEXT_OBJECT", SystemVariable::RC_PICKIT_NEXT_OBJECT)
      .value("RC_PICKIT_GET_PICK_POINT_DATA", SystemVariable::RC_PICKIT_GET_PICK_POINT_DATA)
      .value("RC_PICKIT_CONFIGURE", SystemVariable::RC_PICKIT_CONFIGURE)
      .value("RC_PICKIT_SET_CYLINDER_DIM", SystemVariable::RC_PICKIT_SET_CYLINDER_DIM)
      .value("RC_SAVE_ACTIVE_PRODUCT", SystemVariable::RC_SAVE_ACTIVE_PRODUCT)
      .value("RC_SAVE_ACTIVE_SETUP", SystemVariable::RC_SAVE_ACTIVE_SETUP)
      .value("RC_SAVE_BUILD_BACKGROUND", SystemVariable::RC_SAVE_BUILD_BACKGROUND)
      .value("RC_PICKIT_FIND_CALIB_PLATE", SystemVariable::RC_PICKIT_FIND_CALIB_PLATE)
      .value("RC_PICKIT_SAVE_SCENE", SystemVariable::RC_PICKIT_SAVE_SCENE)
      .value("PICKIT_STATUS", SystemVariable::PICKIT_STATUS)
      .value("PICKIT_VERSION", SystemVariable::PICKIT_VERSION)
      .value("PICKIT_ROBOTTYPE", SystemVariable::PICKIT_ROBOTTYPE)
      .value("PICKIT_ROBOT_MODE", SystemVariable::PICKIT_ROBOT_MODE)
      .value("PICKIT_IDLE_MODE", SystemVariable::PICKIT_IDLE_MODE)
      .value("PICKIT_OBJECT_FOUND", SystemVariable::PICKIT_OBJECT_FOUND)
      .value("PICKIT_NO_OBJECTS", SystemVariable::PICKIT_NO_OBJECTS)
      .value("PICKIT_IMAGE_CAPTURED", SystemVariable::PICKIT_IMAGE_CAPTURED)
      .value("PICKIT_NO_IMAGE_CAPTURED", SystemVariable::PICKIT_NO_IMAGE_CAPTURED)
      .value("PICKIT_EMPTY_ROI", SystemVariable::PICKIT_EMPTY_ROI)
      .value("PICKIT_GET_PICK_POINT_DATA_OK", SystemVariable::PICKIT_GET_PICK_POINT_DATA_OK)
      .value("PICKIT_GET_PICK_POINT_DATA_FAILED", SystemVariable::PICKIT_GET_PICK_POINT_DATA_FAILED)
      .value("PICKIT_CONFIG_OK", SystemVariable::PICKIT_CONFIG_OK)
      .value("PICKIT_CONFIG_FAILED", SystemVariable::PICKIT_CONFIG_FAILED)
      .value("PICKIT_BUILD_BKG_CLOUD_OK", SystemVariable::PICKIT_BUILD_BKG_CLOUD_OK)
      .value("PICKIT_BUILD_BKG_CLOUD_FAILED", SystemVariable::PICKIT_BUILD_BKG_CLOUD_FAILED)
      .value("PICKIT_FIND_CALIB_PLATE_OK", SystemVariable::PICKIT_FIND_CALIB_PLATE_OK)
      .value("PICKIT_FIND_CALIB_PLATE_FAILED", SystemVariable::PICKIT_FIND_CALIB_PLATE_FAILED)
      .value("PICKIT_SAVE_SNAPSHOT_OK", SystemVariable::PICKIT_SAVE_SNAPSHOT_OK)
      .value("PICKIT_SAVE_SNAPSHOT_FAILED", SystemVariable::PICKIT_SAVE_SNAPSHOT_FAILED)
      .value("PICKIT_UNKNOWN_COMMAND", SystemVariable::PICKIT_UNKNOWN_COMMAND)
      .value("PICKIT_TYPE_SQUARE", SystemVariable::PICKIT_TYPE_SQUARE)
      .value("PICKIT_TYPE_RECTANGLE", SystemVariable::PICKIT_TYPE_RECTANGLE)
      .value("PICKIT_TYPE_CIRCLE", SystemVariable::PICKIT_TYPE_CIRCLE)
      .value("PICKIT_TYPE_ELLIPSE", SystemVariable::PICKIT_TYPE_ELLIPSE)
      .value("PICKIT_TYPE_CYLINDER", SystemVariable::PICKIT_TYPE_CYLINDER)
      .value("PICKIT_TYPE_SPHERE", SystemVariable::PICKIT_TYPE_SPHERE)
      .value("PICKIT_TYPE_POINTCLOUD", SystemVariable::PICKIT_TYPE_POINTCLOUD)
      .value("PICKIT_TYPE_BLOB", SystemVariable::PICKIT_TYPE_BLOB)
      .value("PICKIT_X", SystemVariable::PICKIT_X)
      .value("PICKIT_Y", SystemVariable::PICKIT_Y)
      .value("PICKIT_Z", SystemVariable::PICKIT_Z)
      .value("PICKIT_RX", SystemVariable::PICKIT_RX)
      .value("PICKIT_RY", SystemVariable::PICKIT_RY)
      .value("PICKIT_RZ", SystemVariable::PICKIT_RZ)
      .value("PICKIT_P0", SystemVariable::PICKIT_P0)
      .value("PICKIT_P1", SystemVariable::PICKIT_P1)
      .value("PICKIT_P2", SystemVariable::PICKIT_P2)
      .value("PICKIT_P3", SystemVariable::PICKIT_P3)
      .value("PICKIT_P4", SystemVariable::PICKIT_P4)
      .value("PICKIT_P5", SystemVariable::PICKIT_P5)
      .value("PICKIT_P0F", SystemVariable::PICKIT_P0F)
      .value("PICKIT_P1F", SystemVariable::PICKIT_P1F)
      .value("PICKIT_P2F", SystemVariable::PICKIT_P2F)
      .value("PICKIT_P3F", SystemVariable::PICKIT_P3F)
      .value("PICKIT_P4F", SystemVariable::PICKIT_P4F)
      .value("PICKIT_P5F", SystemVariable::PICKIT_P5F)
      .value("PICKIT_M0", SystemVariable::PICKIT_M0)
      .value("PICKIT_M1", SystemVariable::PICKIT_M1)
      .value("ICE_INFO_CONNECTED", SystemVariable::ICE_INFO_CONNECTED)
      .value("ICE_INFO_REQUESTING", SystemVariable::ICE_INFO_REQUESTING)
      .value("ICE_INFO_USING", SystemVariable::ICE_INFO_USING)
      .value("ICE_INFO_VECSIZE", SystemVariable::ICE_INFO_VECSIZE)
      .value("ICE_INFO_MODE_COMM", SystemVariable::ICE_INFO_MODE_COMM)
      .value("ICE_INFO_MODE_CUP", SystemVariable::ICE_INFO_MODE_CUP)
      .value("ICE_INFO_TIME_ICE", SystemVariable::ICE_INFO_TIME_ICE)
      .value("ICE_INFO_TIME_WATER", SystemVariable::ICE_INFO_TIME_WATER)
      .value("ICE_INFO_AMBI_LOW", SystemVariable::ICE_INFO_AMBI_LOW)
      .value("ICE_INFO_AMBI_HIGH", SystemVariable::ICE_INFO_AMBI_HIGH)
      .value("ICE_INFO_TMEP_AMBI", SystemVariable::ICE_INFO_TMEP_AMBI)
      .value("ICE_INFO_TMEP_EVAPO", SystemVariable::ICE_INFO_TMEP_EVAPO)
      .value("ICE_INFO_TMEP_CONDEN", SystemVariable::ICE_INFO_TMEP_CONDEN)
      .value("ICE_STATE_LAST_ICE_NO", SystemVariable::ICE_STATE_LAST_ICE_NO)
      .value("ICE_STATE_LAST_ICE_YES", SystemVariable::ICE_STATE_LAST_ICE_YES)
      .value("ICE_STATE_COMP_WORK", SystemVariable::ICE_STATE_COMP_WORK)
      .value("ICE_STATE_MOTOR_WORK", SystemVariable::ICE_STATE_MOTOR_WORK)
      .value("ICE_STATE_OUT_SOL", SystemVariable::ICE_STATE_OUT_SOL)
      .value("ICE_STATE_CUP_LEVEL", SystemVariable::ICE_STATE_CUP_LEVEL)
      .value("ICE_STATE_COMM_MODE", SystemVariable::ICE_STATE_COMM_MODE)
      .value("ICE_STATE_FULL_ICE", SystemVariable::ICE_STATE_FULL_ICE)
      .value("ICE_STATE_ERR_1", SystemVariable::ICE_STATE_ERR_1)
      .value("ICE_STATE_ERR_2", SystemVariable::ICE_STATE_ERR_2)
      .value("ICE_STATE_ERR_3", SystemVariable::ICE_STATE_ERR_3)
      .value("ICE_STATE_ERR_4", SystemVariable::ICE_STATE_ERR_4)
      .value("ICE_STATE_ERR_CODE", SystemVariable::ICE_STATE_ERR_CODE)
      .value("ICE_STATE_RD", SystemVariable::ICE_STATE_RD)
      .value("SETECH_RDY", SystemVariable::SETECH_RDY)
      .value("SETECH_ALM", SystemVariable::SETECH_ALM)
      .value("SETECH_BUSY", SystemVariable::SETECH_BUSY)
      .value("SETECH_COMP", SystemVariable::SETECH_COMP)
      .value("SETECH_OK", SystemVariable::SETECH_OK)
      .value("SETECH_NG_TRQH", SystemVariable::SETECH_NG_TRQH)
      .value("SETECH_NG_TRQL", SystemVariable::SETECH_NG_TRQL)
      .value("SETECH_NG_ANGH", SystemVariable::SETECH_NG_ANGH)
      .value("SETECH_NG_ANGL", SystemVariable::SETECH_NG_ANGL)
      .value("SETECH_NG_TIME", SystemVariable::SETECH_NG_TIME)
      .value("SETECH_NG_MONI", SystemVariable::SETECH_NG_MONI)
      .value("SETECH_NG_CH1", SystemVariable::SETECH_NG_CH1)
      .value("SETECH_NG_CH2", SystemVariable::SETECH_NG_CH2)
      .value("SETECH_NG_CH4", SystemVariable::SETECH_NG_CH4)
      .value("SETECH_NG_CH8", SystemVariable::SETECH_NG_CH8)
      .value("SETECH_NG_CH16", SystemVariable::SETECH_NG_CH16)
      .value("SD_NO_ARC_STATE", SystemVariable::SD_NO_ARC_STATE)
      .value("SD_DWELD_ARC", SystemVariable::SD_DWELD_ARC)
      .value("SD_DWELD_TOUCH", SystemVariable::SD_DWELD_TOUCH)
      .value("SD_DWELD_A", SystemVariable::SD_DWELD_A)
      .value("SD_DWELD_V", SystemVariable::SD_DWELD_V)
      .value("SD_DWELD_F", SystemVariable::SD_DWELD_F)
      .value("SD_DWELD_SET_A", SystemVariable::SD_DWELD_SET_A)
      .value("SD_CONV_POS_TICK", SystemVariable::SD_CONV_POS_TICK)
      .value("SD_CONV_VEL_TICK", SystemVariable::SD_CONV_VEL_TICK)
      .export_values();

  py::enum_<GripperModel>(m, "GripperModel")
      .value("Robotiq_Hand_E", GripperModel::Robotiq_Hand_E)
      .value("Robotiq_2F_85", GripperModel::Robotiq_2F_85)
      .value("Robotiq_2F1_40", GripperModel::Robotiq_2F1_40)
      .value("Robotiq_E_Pick", GripperModel::Robotiq_E_Pick)
      .value("Robotis_RH_P12_RN", GripperModel::Robotis_RH_P12_RN)
      .value("JRT_JEGB_4285", GripperModel::JRT_JEGB_4285)
      .value("JRT_JEGB_42140", GripperModel::JRT_JEGB_42140)
      .value("DH_AG_95", GripperModel::DH_AG_95)
      .value("Setech_NutRunner", GripperModel::Setech_NutRunner)
      .value("OnRobot2FG7", GripperModel::OnRobot2FG7)
      .value("JRT_JEGG", GripperModel::JRT_JEGG)
      .value("RB_5FDG", GripperModel::RB_5FDG)
      .value("OnRobot_RG2", GripperModel::OnRobot_RG2)
      .value("OnRobot_RG6", GripperModel::OnRobot_RG6)
      .value("OnRobot_VG10", GripperModel::OnRobot_VG10)
      .value("OnRobot_VGC10", GripperModel::OnRobot_VGC10)
      .value("OnRobot_3FG15", GripperModel::OnRobot_3FG15)
      .value("OnRobot_Sander", GripperModel::OnRobot_Sander)
      .value("OnRobot_VGP20", GripperModel::OnRobot_VGP20)
      .value("OnRobot_MG10", GripperModel::OnRobot_MG10)
      .value("OnRobot_SoftGripper", GripperModel::OnRobot_SoftGripper)
      .value("JRT_JEGC", GripperModel::JRT_JEGC)
      .value("Schunk_Coact", GripperModel::Schunk_Coact)
      .value("Baumer_Laser_Sensor", GripperModel::Baumer_Laser_Sensor)
      .value("Mighty_12LF", GripperModel::Mighty_12LF)
      .value("JRT_JEGH_3520", GripperModel::JRT_JEGH_3520);

  py::enum_<GripperConnectionPoint>(m, "GripperConnectionPoint")
      .value("ToolFlange", GripperConnectionPoint::ToolFlange)
      .value("ControlBox", GripperConnectionPoint::ControlBox)
      .value("ToolFlange_Advanced", GripperConnectionPoint::ToolFlange_Advanced);

  py::class_<SystemState> ss(m, "SystemState");
  py::class_<SystemState::Data>(ss, "Data")
      .def_readonly("time", &SystemState::Data::time, R"pbdoc(
Basic timer. (Unit: sec)
)pbdoc")
      .def_property_readonly(
          "jnt_ref", [](const SystemState::Data& data) { return py::array_t<float>(6, data.jnt_ref); }, R"pbdoc(
Reference (desired) joint position. (Unit: deg)

0 = Base / 1 = Shoulder / 2 = Elbow / 3 = Wrist1 / 4 = Wrist2 / 5 = Wrist3
)pbdoc")
      .def_property_readonly(
          "jnt_ang", [](const SystemState::Data& data) { return py::array_t<float>(6, data.jnt_ang); }, R"pbdoc(
Measured joint position. (Unit: deg)

0 = Base / 1 = Shoulder / 2 = Elbow / 3 = Wrist1 / 4 = Wrist2 / 5 = Wrist3

These values do not change in simulation mode.
)pbdoc")
      .def_property_readonly(
          "jnt_cur", [](const SystemState::Data& data) { return py::array_t<float>(6, data.jnt_cur); }, R"pbdoc(
Measured joint current. (Unit: Amp)

0 = Base / 1 = Shoulder / 2 = Elbow / 3 = Wrist1 / 4 = Wrist2 / 5 = Wrist3

Note
----
You can get joint torque by multiplying torque constant.
)pbdoc")
      .def_property_readonly(
          "tcp_ref", [](const SystemState::Data& data) { return py::array_t<float>(6, data.tcp_ref); }, R"pbdoc(
TCP posture info based on reference-joint-angles (unit: mm & degree)

0 = X / 1 = Y / 2 = Z / 3 = Rx / 4 = Ry / 5 = Rz
)pbdoc")
      .def_property_readonly(
          "tcp_pos", [](const SystemState::Data& data) { return py::array_t<float>(6, data.tcp_pos); }, R"pbdoc(
TCP posture info based on encoder-joint-angles (unit: mm & degree)

0 = X / 1 = Y / 2 = Z / 3 = Rx / 4 = Ry / 5 = Rz

Warning
-------
It is being transmitted overwritten based on the current reference.
)pbdoc")
      .def_property_readonly(
          "analog_in", [](const SystemState::Data& data) { return py::array_t<float>(4, data.analog_in); }, R"pbdoc(
Control box analog input measurement information of each channel (unit: Voltage)

Channel number: 0~3
)pbdoc")
      .def_property_readonly(
          "analog_out", [](const SystemState::Data& data) { return py::array_t<float>(4, data.analog_out); }, R"pbdoc(
Control box analog output information of each channel (unit: Voltage)

Channel number: 0~3
)pbdoc")
      .def_property_readonly(
          "digital_in", [](const SystemState::Data& data) { return py::array_t<int>(6, data.digital_in); }, R"pbdoc(
Control box digital input measurement information of each channel (value: 0 or 1)

Channel number: 0~15
)pbdoc")
      .def_property_readonly(
          "digital_out", [](const SystemState::Data& data) { return py::array_t<int>(6, data.digital_out); }, R"pbdoc(
Control box digital output information of each channel (value: 0 or 1)

Channel number: 0~15
)pbdoc")
      .def_property_readonly(
          "jnt_temperature", [](const SystemState::Data& data) { return py::array_t<float>(6, data.jnt_temperature); },
          R"pbdoc(
Measured temperature of each joint. (unit: celsius)

0 = Base / 1 = Shoulder / 2 = Elbow / 3 = Wrist1 / 4 = Wrist2 / 5 = Wrist3
)pbdoc")
      .def_readonly("task_pc", &SystemState::Data::task_pc, R"pbdoc(
Target program counter position during [STEP] function.
)pbdoc")
      .def_readonly("task_repeat", &SystemState::Data::task_repeat, R"pbdoc(
Target program execution number in [PLAY] page.
)pbdoc")
      .def_readonly("task_run_id", &SystemState::Data::task_run_id, R"pbdoc(
Running program counter position.
)pbdoc")
      .def_readonly("task_run_num", &SystemState::Data::task_run_num, R"pbdoc(
Current program execution number in [PLAY] page.
)pbdoc")
      .def_readonly("task_run_time", &SystemState::Data::task_run_time, R"pbdoc(
Time since the program started (unit: second)
)pbdoc")
      .def_readonly("task_state", &SystemState::Data::task_state, R"pbdoc(
Basic state of 'Program Execution'

* 1 = Program not run / Idle
* 3 = Program is running
* 2 = Program is running + but ‘Paused’state
)pbdoc")
      .def_readonly("default_speed", &SystemState::Data::default_speed, R"pbdoc(
Default speed multiplier value of robot motion (=speed bar in UI) (value: 0 ~ 1)
)pbdoc")
      .def_readonly("robot_state", &SystemState::Data::robot_state, R"pbdoc(
Move (motion) state

* 1 = No motion command / Idle
* 3 = Executing motion command(s)
* 5 = No motion (Move) command + but executing Conveyor or Force control mode
* 60+index = Under MovePB/ITPL/Pro command / index is passing waypoint number
)pbdoc")
      .def_readonly("information_chunk_1", &SystemState::Data::information_chunk_1, R"pbdoc(
Information chunk to deliver various state information (power and others). It consists of a combination of bits.

* (information_chunk_1 >> 0) & 0b01 = Control Box's 48V input state
* (information_chunk_1 >> 1) & 0b01 = Control Box's 48V output state
* (information_chunk_1 >> 2) & 0b01 = Control Box's 24V input state
* (information_chunk_1 >> 3) & 0b01 = Control Box's E-Stop state 1
* (information_chunk_1 >> 4) & 0b01 = Control Box's User Switch state
* (information_chunk_1 >> 5) & 0b01 = Control Box's E-Stop state 2
* (information_chunk_1 >> 6) & 0b01 = Whether power is applied to the robot arm
* (information_chunk_1 >> 7) & 0b01 = TFB’s Direct teaching button is pressed
* (information_chunk_1 >> 30) & 0b01 = Program Load state
* (Whenever the Program load process is successful, 1 and 0 are continuously converted.)
* (information_chunk_1 >> 31) & 0b01 = Program Transmit state (via TCP/IP Tablet UI, not for user)
)pbdoc")
      .def_property_readonly(
          "reserved_1", [](const SystemState::Data& data) { return py::array_t<float>(6, data.reserved_1); }, R"pbdoc(
Reserved / Not used
)pbdoc")
      .def_property_readonly(
          "jnt_info", [](const SystemState::Data& data) { return py::array_t<int>(6, data.jnt_info); }, R"pbdoc(
Basic state of each joint.

0 = Base / 1 = Shoulder / 2 = Elbow / 3 = Wrist1 / 4 = Wrist2 / 5 = Wrist3

* Each int (4byte) consists of a combination of bits.
* (jnt_info[#] >> 0) & 0b01 = Joint #'s FET state
* (jnt_info[#] >> 1) & 0b01 = Joint #'s RUN state
* (jnt_info[#] >> 2) & 0b01 = Joint #'s INIT state
* (jnt_info[#] >> 3) & 0b01 = Joint #'s MODE state
* (jnt_info[#] >> 4) & 0b01 = Joint #'s encoder state (Nonius err)
* (jnt_info[#] >> 5) & 0b01 = Joint #'s encoder state (LowBatt err)
* (jnt_info[#] >> 6) & 0b01 = Joint #'s encoder state (Calibration mode)
* (jnt_info[#] >> 7) & 0b01 = Joint #'s encoder state (Multi-turn err)
* (jnt_info[#] >> 8) & 0b01 = Joint #'s Error state (JAM err)
* (jnt_info[#] >> 9) & 0b01 = Joint #'s Error state (CUR err)
* (jnt_info[#] >> 10) & 0b01 = Joint #'s Error state (BIG err)
* (jnt_info[#] >> 11) & 0b01 = Joint #'s Error state (INP err)
* (jnt_info[#] >> 12) & 0b01 = Joint #'s Error state (FLT err)
* (jnt_info[#] >> 13) & 0b01 = Joint #'s Error state (TMP err)
* (jnt_info[#] >> 14) & 0b01 = Joint #'s Error state (PS1 err)
* (jnt_info[#] >> 15) & 0b01 = Joint #'s Error state (PS2 err)
)pbdoc")
      .def_readonly("collision_detect_onoff", &SystemState::Data::collision_detect_onoff, R"pbdoc(
Out collision detection On/Off State (1=On / 0 = Off)
)pbdoc")
      .def_readonly("is_freedrive_mode", &SystemState::Data::is_freedrive_mode, R"pbdoc(
Free-drive (Gravity-compensation) On/Off State (1=On / 0 = Off)
)pbdoc")
      .def_readonly("real_vs_simulation_mode", &SystemState::Data::real_vs_simulation_mode, R"pbdoc(
Mode of operation: Simulation mode=1 / Real Robot mode=0
)pbdoc")
      .def_readonly("init_state_info", &SystemState::Data::init_state_info, R"pbdoc(
Robot arm activation (Initialization) stage info (0 -> 6)

* 0: default
* 1: Power check
* 2: Device check
* 3: Servo Initialization check
* 4: Parameter check
* 5: Payload check
* 6: Activation done
)pbdoc")
      .def_readonly("init_error", &SystemState::Data::init_error, R"pbdoc(
Error code during the arm activation (return value for UI)
)pbdoc")
      .def_property_readonly(
          "tfb_analog_in", [](const SystemState::Data& data) { return py::array_t<float>(2, data.tfb_analog_in); },
          R"pbdoc(
Robot-Tool-Flange analog input measurement information of each channel (unit: Voltage)
)pbdoc")
      .def_property_readonly(
          "tfb_digital_in", [](const SystemState::Data& data) { return py::array_t<int>(2, data.tfb_digital_in); },
          R"pbdoc(
Robot-Tool-Flange digital input measurement information of each channel (value: 0 or 1)
)pbdoc")
      .def_property_readonly(
          "tfb_digital_out", [](const SystemState::Data& data) { return py::array_t<int>(2, data.tfb_digital_out); },
          R"pbdoc(
Robot-Tool-Flange digital output information of each channel (value: 0 or 1)
)pbdoc")
      .def_readonly("tfb_voltage_out", &SystemState::Data::tfb_voltage_out, R"pbdoc(
Robot-Tool-Flage output voltage level (unit: Voltage)
)pbdoc")
      .def_readonly("op_stat_collision_occur", &SystemState::Data::op_stat_collision_occur, R"pbdoc(
Whether out-collision is detected (0 or 1)
)pbdoc")
      .def_readonly("op_stat_sos_flag", &SystemState::Data::op_stat_sos_flag, R"pbdoc(
Robot Arm device error code during operation.

* 0 = None
* 1 = Encoder err (PVL)
* 2 = CPU err
* 3 = Big err
* 4 = Input err
* 5 = JAM err
* 6 = Over current err
* 7 = Position bound err
* 8 = Mode err
* 9 = Match err
* 10 = Over current/Low voltage err
* 11 = Temperature err
* 12 = Speed over err
)pbdoc")
      .def_readonly("op_stat_self_collision", &SystemState::Data::op_stat_self_collision, R"pbdoc(
Whether self-collision is detected (0 or 1)
)pbdoc")
      .def_readonly("op_stat_soft_estop_occur", &SystemState::Data::op_stat_soft_estop_occur, R"pbdoc(
Pause state flag (0 or 1)
)pbdoc")
      .def_readonly("op_stat_ems_flag", &SystemState::Data::op_stat_ems_flag, R"pbdoc(
Software (kinematics) emergency stop situation

0 = None / 1 = Arm Stretch / 2= Cartesian Limit / 3=Joint Limit / 4=Un-solvable
)pbdoc")
      .def_readonly("information_chunk_2", &SystemState::Data::information_chunk_2, R"pbdoc(
Information chunk to deliver various state information. It consists of a combination of bits.

* (information_chunk_2 >> 0) & 0b11 = Config digital input 16 (0 or 1) (Not for user)
* (information_chunk_2 >> 2) & 0b1111111111111111 = Target welding voltage * 100
)pbdoc")
      .def_readonly("information_chunk_3", &SystemState::Data::information_chunk_3, R"pbdoc(
Information chunk to deliver various state information. It consists of a combination of bits.

* (information_chunk_3 >> 0) & 0b11 = Config digital input 17 (0 or 1) (Not for user)
)pbdoc")
      .def_property_readonly(
          "inbox_trap_flag", [](const SystemState::Data& data) { return py::array_t<int>(2, data.inbox_trap_flag); },
          R"pbdoc(
Whether or not detected by the Inbox # check-function.

# = In Box number: 0 or 1
)pbdoc")
      .def_property_readonly(
          "inbox_check_mode", [](const SystemState::Data& data) { return py::array_t<int>(2, data.inbox_check_mode); },
          R"pbdoc(
Check-function mode of Inbox #.

# = In Box number: 0 or 1
0 = None / 1 = Check Tool Flange center / 2 = Check TCP / 3 = Check Tool Box / 4 = Check all
)pbdoc")
      .def_readonly("eft_fx", &SystemState::Data::eft_fx, R"pbdoc(
External F/T force sensor value. Fx (Unit: N)
)pbdoc")
      .def_readonly("eft_fy", &SystemState::Data::eft_fy, R"pbdoc(
External F/T force sensor value. Fy (Unit: N)
)pbdoc")
      .def_readonly("eft_fz", &SystemState::Data::eft_fz, R"pbdoc(
External F/T force sensor value. Fz (Unit: N)
)pbdoc")
      .def_readonly("eft_mx", &SystemState::Data::eft_mx, R"pbdoc(
External F/T torque sensor value. Mx (Unit: Nm)
)pbdoc")
      .def_readonly("eft_my", &SystemState::Data::eft_my, R"pbdoc(
External F/T torque sensor value. My (Unit: Nm)
)pbdoc")
      .def_readonly("eft_mz", &SystemState::Data::eft_mz, R"pbdoc(
External F/T torque sensor value. Mz (Unit: Nm)
)pbdoc")
      .def_readonly("information_chunk_4", &SystemState::Data::information_chunk_4, R"pbdoc(
Information chunk to deliver various state information. It consists of a combination of bits.

* (information_chunk_4 >> 0) & 0b11 = No-Arc Function On/Off (0 or 1)
* (information_chunk_4 >> 2) & 0b111111 = Selected Tool List number
* (information_chunk_4 >> 8) & 0b11 = External Joint (External axis) Jog On/Off (0 or 1)
* (information_chunk_4 >> 10) & 0b01 = Tool Flange Digital Input 2
* (information_chunk_4 >> 11) & 0b01 = Tool Flange Digital Input 3
* (information_chunk_4 >> 12) & 0b01 = Tool Flange Digital Input 4
* (information_chunk_4 >> 13) & 0b01 = Tool Flange Digital Input 5
* (information_chunk_4 >> 14) & 0b01 = Arc Light On state (Not for user)
* (information_chunk_4 >> 15) & 0b1111111111111 = Target welding current * 10
* (information_chunk_4 >> 28) & 0b11 = Target welding voltage option (0 or 1)
)pbdoc")
      .def_property_readonly(
          "extend_io1_analog_in",
          [](const SystemState::Data& data) { return py::array_t<float>(4, data.extend_io1_analog_in); }, R"pbdoc(
Extended I/O board analog input measurement information of each channel (unit: Voltage)

Channel number: 0~3
)pbdoc")
      .def_property_readonly(
          "extend_io1_analog_out",
          [](const SystemState::Data& data) { return py::array_t<float>(4, data.extend_io1_analog_out); }, R"pbdoc(
Extended I/O board analog output information of each channel (unit: Voltage)

Channel number: 0~3
)pbdoc")
      .def_readonly("extend_io1_digital_info", &SystemState::Data::extend_io1_digital_info, R"pbdoc(
Extended I/O board digital input/output information. It consists of a combination of bits

* (extend_io1_digital_info >> 0) & 0b01 = Extend I/O digital input # 0
* (extend_io1_digital_info >> 1) & 0b01 = Extend I/O digital input # 1
* ..
* (extend_io1_digital_info >> 15) & 0b01 = Extend I/O digital input # 15
* (extend_io1_digital_info >> 16) & 0b01 = Extend I/O digital output # 0
* (extend_io1_digital_info >> 17) & 0b01 = Extend I/O digital output # 1
* ..
* (extend_io1_digital_info >> 31) & 0b01 = Extend I/O digital input # 15
)pbdoc")
      .def_property_readonly(
          "aa_joint_ref", [](const SystemState::Data& data) { return py::array_t<float>(6, data.aa_joint_ref); },
          R"pbdoc(
Reference angle of each external-joint (auxiliary joint). (unit: degree)

External joint number: 0~5
)pbdoc")
      .def_readonly("safety_board_stat_info", &SystemState::Data::safety_board_stat_info, R"pbdoc(
Data chunk about the control box safety board
)pbdoc");
  ss.def_readonly("sdata", &SystemState::sdata);
}