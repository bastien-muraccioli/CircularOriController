#include "CollisionBenchmarkController_PickPlace.h"

void CollisionBenchmarkController_PickPlace::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_PickPlace::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Position");
  ctl.postureTask->target(ctl.postureHome);
  ctl.postureTask->stiffness(0.5);
  ctl.postureTask->weight(0.1);
  ctl.eeTask->reset();
  ctl.eeTask->positionTask->position(ctl.taskPosPickPlaceDownRight);
  ctl.eeTask->positionTask->stiffness(400);
  ctl.eeTask->positionTask->weight(10000);
  
  ctl.solver().addTask(ctl.eeTask);
}

bool CollisionBenchmarkController_PickPlace::run(mc_control::fsm::Controller & ctl_)
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

  if (ctl.eeTask->positionTask->eval().norm() < 0.01)
  {
    switch (state_) 
    {
      case DR:
        ctl.eeTask->positionTask->position(ctl.taskPosPickPlaceUpRight);
        state_ = UR;
        break;
      case UR:
        ctl.eeTask->positionTask->position(ctl.taskPosPickPlaceUpLeft);
        state_ = UL;
        break;
      case UL:
        ctl.eeTask->positionTask->position(ctl.taskPosPickPlaceDownLeft);
        state_ = DL;
        break;
      case DL:
        ctl.eeTask->positionTask->position(ctl.taskPosPickPlaceUpLeft);
        state_ = LU;
        break;
      case LU:
        ctl.eeTask->positionTask->position(ctl.taskPosPickPlaceUpRight);
        state_ = RU;
        break;
      case RU:
        ctl.eeTask->positionTask->position(ctl.taskPosPickPlaceDownRight);
        state_ = DR;
        break;
    }
  }
  return false;
}

void CollisionBenchmarkController_PickPlace::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.solver().removeTask(ctl.eeTask);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_PickPlace", CollisionBenchmarkController_PickPlace)
