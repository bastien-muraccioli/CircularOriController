#include "CollisionBenchmarkController_FirstJointRotation_InitialPose.h"

void CollisionBenchmarkController_FirstJointRotation_InitialPose::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_FirstJointRotation_InitialPose::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.postureTask->target(ctl.postureFirstJointRotation);
  ctl.postureTask->stiffness(1);
  ctl.postureTask->damping(2);
}

bool CollisionBenchmarkController_FirstJointRotation_InitialPose::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  if (ctl.postureTask->eval().norm() < 0.1)
  {
    output("OK");
    return true;
  }

  return false;
}

void CollisionBenchmarkController_FirstJointRotation_InitialPose::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_FirstJointRotation_InitialPose", CollisionBenchmarkController_FirstJointRotation_InitialPose)
