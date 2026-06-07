#include "CollisionBenchmarkController_Initial.h"

void CollisionBenchmarkController_Initial::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.postureTask->stiffness(1);
  ctl.postureTask->damping(2);
  ctl.postureTask->target(ctl.postureHome);
  ctl.solver().removeTask(ctl.eeTask);
  task_achieved_ = false;
}

bool CollisionBenchmarkController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  if(ctl.datastore().has("Obstacle detected"))
  {
    if(ctl.datastore().get<bool>("Obstacle detected"))
    {
      ctl.datastore().get<bool>("Obstacle detected") = false;
    }
  }
  if(ctl.postureTask->eval().norm() < 0.05 && !task_achieved_)
  {
    task_achieved_ = true;
    mc_rtc::log::success("[CollisionBenchmarkController] Get back to initial posture");
  }
  return false;
}

void CollisionBenchmarkController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_Initial", CollisionBenchmarkController_Initial)
