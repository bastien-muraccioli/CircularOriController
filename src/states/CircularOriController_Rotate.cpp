#include "CircularOriController_Rotate.h"

#include "../CircularOriController.h"

void CircularOriController_Rotate::configure(const mc_rtc::Configuration & config) {}

void CircularOriController_Rotate::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
  // Activate feedback from external forces estimator (safer)
  if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Activate force sensor usage if not used yet
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", 10.0);

  // ctl.compPostureTask->makeCompliant(false);

  ctl.datastore().assign<std::string>("ControlMode", "Position");

}

bool CircularOriController_Rotate::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
  if (ctl.compPostureTask->eval().norm() < 0.1)
  {
    if(need_home_)
    {
      ctl.compPostureTask->target(ctl.postureHome);
      need_home_ = false;
    }
    else
    {
      switch (state_) 
      {
        case 0:
          ctl.compPostureTask->target(ctl.postureUp);
          state_ = 1;
          break;
        case 1:
          ctl.compPostureTask->target(ctl.postureDown);
          state_ = 2;
          break;
        case 2:
          ctl.compPostureTask->target(ctl.postureRight);
          state_ = 3;
          break;
        case 3:
          ctl.compPostureTask->target(ctl.postureLeft);
          state_ = 0;
          break;
      }
      need_home_ = true;
    }
  }
// output("OK");
//   return true;
  return false;
}

void CircularOriController_Rotate::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
}

EXPORT_SINGLE_STATE("CircularOriController_Rotate", CircularOriController_Rotate)