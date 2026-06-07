#pragma once

#include <mc_control/fsm/State.h>
#include "../CollisionBenchmarkController.h"

#include <RBDyn/Coriolis.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

struct CollisionBenchmarkController_ReactionSimple : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::vector<bool> joint_stop_;
  int jointNumber_;
  bool all_joint_stop_;
  double counter_ = 0.0;
  double stop_time_ = 2.0; // Stop for 2 seconds
  bool task_achieved_ = false;
};
