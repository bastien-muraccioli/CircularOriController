#include "CollisionBenchmarkController_PickPlace_InitialPose.h"

void CollisionBenchmarkController_PickPlace_InitialPose::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_PickPlace_InitialPose::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Position");
  ctl.postureTask->target(ctl.postureHome);
  ctl.postureTask->stiffness(0.5);
  ctl.postureTask->weight(0.1);
  ctl.eeTask->reset();
  ctl.eeTask->positionTask->position(ctl.taskPosPickPlaceDownRight);
  ctl.eeTask->positionTask->stiffness(1);
  ctl.eeTask->positionTask->weight(10000);
  ctl.eeTask->orientationTask->orientation(ctl.taskOriPickPlace.toRotationMatrix());
  ctl.eeTask->orientationTask->stiffness(1);
  ctl.eeTask->orientationTask->weight(1000);
  
  ctl.solver().addTask(ctl.eeTask);
}

bool CollisionBenchmarkController_PickPlace_InitialPose::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  if (ctl.eeTask->eval().norm() < 0.03)
  {
    output("OK");
    return true;
  }
  return false;
}

void CollisionBenchmarkController_PickPlace_InitialPose::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.solver().removeTask(ctl.eeTask);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_PickPlace_InitialPose", CollisionBenchmarkController_PickPlace_InitialPose)
