#include "CollisionBenchmarkController_LastJointsRotation.h"

void CollisionBenchmarkController_LastJointsRotation::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_LastJointsRotation::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.postureTask->target(ctl.postureHome);
  ctl.postureTask->stiffness(400);
}

bool CollisionBenchmarkController_LastJointsRotation::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);

  if(ctl.datastore().get<bool>("Obstacle detected"))
  {
    ctl.postureTask->reset();
    ctl.postureTask->stiffness(500);
    output("OK");
    return true;
  }

  if (ctl.postureTask->eval().norm() < 0.1)
  {
    if(need_home_)
    {
      ctl.postureTask->target(ctl.postureHome);
      need_home_ = false;
    }
    else
    {
      switch (state_) 
      {
        case 0:
          ctl.postureTask->target(ctl.postureUp);
          state_ = 1;
          break;
        case 1:
          ctl.postureTask->target(ctl.postureDown);
          state_ = 2;
          break;
        case 2:
          ctl.postureTask->target(ctl.postureRight);
          state_ = 3;
          break;
        case 3:
          ctl.postureTask->target(ctl.postureLeft);
          state_ = 0;
          break;
      }
      need_home_ = true;
    }
  }
  return false;
}

void CollisionBenchmarkController_LastJointsRotation::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_LastJointsRotation", CollisionBenchmarkController_LastJointsRotation)
