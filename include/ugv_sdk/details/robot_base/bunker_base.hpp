/*
 * bunker_base.hpp
 *
 * Created on: Jul 14, 2021 23:05
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef BUNKER_BASE_HPP
#define BUNKER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "ugv_sdk/details/interface/bunker_interface.hpp"
#include "ugv_sdk/details/robot_base/agilex_base.hpp"

namespace westonrobot {
template <typename ParserType>
class BunkerBase : public AgilexBase<ParserType>, public BunkerInterface {
 public:
  BunkerBase() : AgilexBase<ParserType>(){};
  ~BunkerBase() = default;

  // set up connection
  bool Connect(std::string can_name) override {
    return AgilexBase<ParserType>::Connect(can_name);
  }

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel) override {
    AgilexBase<ParserType>::SendMotionCommand(linear_vel, angular_vel, 0.0,
                                              0.0);
  }

  // get robot state
  BunkerCoreState GetRobotState() override {
    auto state = AgilexBase<ParserType>::GetRobotCoreStateMsgGroup();

    BunkerCoreState bunker_state;
    bunker_state.time_stamp = state.time_stamp;
    bunker_state.motion_state_stamp = state.motion_state_stamp;
    bunker_state.system_state = state.system_state;
    bunker_state.motion_state = state.motion_state;
    bunker_state.rc_state = state.rc_state;
    return bunker_state;
  }

  BunkerActuatorState GetActuatorState() override {
    auto actuator = AgilexBase<ParserType>::GetActuatorStateMsgGroup();

    BunkerActuatorState bunker_actuator{};
    bunker_actuator.time_stamp = actuator.time_stamp;
    // v2: the drivers don't necessarily start at motor_id 0 (Bunker Pro: 1 and 2), so take the
    // first two that have reported instead of slots 0 and 1
    int n = 0;
    for (int id = 0; id < AGX_MAX_ACTUATOR_NUM && n < 2; ++id) {
      if (!actuator.actuator_hs_received[id] && !actuator.actuator_ls_received[id]) continue;
      bunker_actuator.actuator_hs_state[n] = actuator.actuator_hs_state[id];
      bunker_actuator.actuator_ls_state[n] = actuator.actuator_ls_state[id];
      bunker_actuator.actuator_hs_state[n].motor_id = id;
      bunker_actuator.actuator_ls_state[n].motor_id = id;
      ++n;
    }
    bunker_actuator.actuator_num = n;
    // v1 robots: slots 0 and 1, as before
    for (int i = 0; i < 2; ++i) {
      bunker_actuator.actuator_state[i] = actuator.actuator_state[i];
    }
    return bunker_actuator;
  }
};
}  // namespace westonrobot

#include "ugv_sdk/details/protocol_v1/protocol_v1_parser.hpp"
#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
using BunkerBaseV1 = BunkerBase<BunkerProtocolV1Parser>;
using BunkerBaseV2 = BunkerBase<ProtocolV2Parser>;
}  // namespace westonrobot

#endif /* BUNKER_BASE_HPP */
