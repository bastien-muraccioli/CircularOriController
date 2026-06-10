#include "CollisionBenchmarkController_Forward.h"

void CollisionBenchmarkController_Forward::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_Forward::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Position");
  ctl.postureTask->target(ctl.postureHome);
  ctl.postureTask->stiffness(0.5);
  ctl.postureTask->weight(0.1);
  ctl.eeTask->reset();
  ctl.eeTask->positionTask->position(ctl.taskPosHome);
  ctl.eeTask->positionTask->stiffness(100);
  ctl.eeTask->orientationTask->orientation(ctl.taskOriHome.toRotationMatrix());
  ctl.eeTask->orientationTask->stiffness(20);
  ctl.eeTask->positionTask->weight(10000);
  
  ctl.solver().addTask(ctl.eeTask);
}

bool CollisionBenchmarkController_Forward::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  if(ctl.datastore().get<bool>("Obstacle detected"))
  {
    ctl.postureTask->reset();
    ctl.eeTask->reset();
    ctl.postureTask->stiffness(500);
    output("OK");
    return true;
  }

  if (ctl.eeTask->positionTask->eval().norm() < 0.05)
  {
   
    if(need_home_)
    {
      ctl.eeTask->positionTask->position(ctl.taskPosHome);
      need_home_ = false;
      ctl.eeTask->positionTask->stiffness(10);
    }
    else
    {
      ctl.eeTask->positionTask->position(ctl.taskPosForward);
      need_home_ = true;
      ctl.eeTask->positionTask->stiffness(100);
    }
  }
  return false;
}

void CollisionBenchmarkController_Forward::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.solver().removeTask(ctl.eeTask);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_Forward", CollisionBenchmarkController_Forward)
