#include "CollisionBenchmarkController_ReactionSimple.h"

void CollisionBenchmarkController_ReactionSimple::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_ReactionSimple::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  jointNumber_ = ctl.robot().refJointOrder().size();
  ctl.postureTask->reset();
  ctl.postureTask->stiffness(500);
  ctl.postureTask->damping(500);
  ctl.postureTask->refAccel(Eigen::VectorXd::Zero(jointNumber_));
  ctl.postureTask->refVel(Eigen::VectorXd::Zero(jointNumber_));
  joint_stop_.resize(jointNumber_);
  
  for(int i = 0; i < jointNumber_; i++)
  {
    joint_stop_[i] = false;
  }
}

bool CollisionBenchmarkController_ReactionSimple::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  auto & robot = ctl.robot();
  auto & rjo = robot.refJointOrder();

  all_joint_stop_ = true;
  for(int i = 0; i < jointNumber_; i++)
  {
    double velocity = robot.alpha()[robot.jointIndexByName(rjo[i])][0];
    if(velocity > 0.001 && !joint_stop_[i])
    {
      ctl.postureTask->reset();
      ctl.postureTask->stiffness(500);
      ctl.postureTask->damping(500);
      all_joint_stop_ = false;
    }
    else
    {
      joint_stop_[i] = true;
    }
  }

  if(all_joint_stop_)
  {
    counter_ += ctl.timeStep;
    if(counter_ >= stop_time_)
    {
      task_achieved_ = true;
    }
  }
  else
  {
    counter_ = 0.0;
  }

  if(task_achieved_)
  {
    mc_rtc::log::info("All joint stopped");
    output("OK");
    return true;
  }

  return false;
}

void CollisionBenchmarkController_ReactionSimple::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_ReactionSimple", CollisionBenchmarkController_ReactionSimple)
