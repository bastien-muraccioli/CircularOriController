#pragma once

#include <mc_control/fsm/State.h>
#include "../CollisionBenchmarkController.h"

struct CollisionBenchmarkController_LastJointsRotation : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  int state_ = 0;
  double reset_plot_timer = 0.0;
  double reset_plot_max_time = 30.0; // 30 seconds
  bool need_home_ = false;
};
