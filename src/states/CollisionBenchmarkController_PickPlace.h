#pragma once

#include <mc_control/fsm/State.h>
#include "../CollisionBenchmarkController.h"

#define DR 0
#define UR 1
#define UL 2
#define DL 3
#define LU 4
#define RU 5


struct CollisionBenchmarkController_PickPlace : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  int state_ = DR;
};
