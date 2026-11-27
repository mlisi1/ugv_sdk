/*
 * bunker_interface.hpp
 *
 * Created on: Jul 14, 2021 23:04
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef BUNKER_INTERFACE_HPP
#define BUNKER_INTERFACE_HPP

#include <string>

#include "ugv_sdk/details/interface/agilex_message.h"
#include "ugv_sdk/details/interface/robot_common_interface.hpp"

namespace westonrobot {
struct BunkerCoreState {
  SdkTimePoint time_stamp;          // last core message of any type
  SdkTimePoint motion_state_stamp;  // last motion state message

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  RcStateMessage rc_state;
};

struct BunkerActuatorState {
  SdkTimePoint time_stamp;

  // actuator state (v2): the drivers that have reported, in ascending motor_id order (the
  // Bunker Pro's two drivers are motor_id 1 and 2, CAN 0x252/0x253 and 0x262/0x263).
  // actuator_num = how many of the slots are valid (0..2); each slot keeps its real motor_id.
  int actuator_num;
  ActuatorHSStateMessage actuator_hs_state[2];
  ActuatorLSStateMessage actuator_ls_state[2];
  // - for v1 robots only
  ActuatorStateMessageV1 actuator_state[2];
};

struct BunkerInterface {
  virtual ~BunkerInterface() = default;

  virtual void SetMotionCommand(double linear_vel, double angular_vel) = 0;

  // get robot state
  virtual BunkerCoreState GetRobotState() = 0;
  virtual BunkerActuatorState GetActuatorState() = 0;
};
}  // namespace westonrobot

#endif /* BUNKER_INTERFACE_HPP */
