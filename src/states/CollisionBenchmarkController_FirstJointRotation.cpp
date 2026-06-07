#include "CollisionBenchmarkController_FirstJointRotation.h"
#include <mc_rtc/logging.h>
#include <RBDyn/MultiBodyConfig.h>
#include <Eigen/src/Core/Matrix.h>
#include <cstdlib>

void CollisionBenchmarkController_FirstJointRotation::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_FirstJointRotation::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();
  
  ctl.postureTask->target(ctl.postureFirstJointRotation);
  ctl.postureTask->stiffness(400);
  velocity_ = Eigen::VectorXd::Zero(jointNumber);
  accel_ = Eigen::VectorXd::Zero(jointNumber);
  q_d_ = ctl.postureFirstJointRotation;
  
  auto it = ctl.postureFirstJointRotation.begin();
  q_min_ = it->second[0];
  q_d_zero_ = q_min_;
  auto it2 = ctl.postureFirstJointRotation_end.begin();
  q_max_ = it2->second[0];
  delta_q_ = std::abs(q_max_ - q_min_);

  // Dynamically calculate the total time that is determined by target position and speed limits
  total_time_ = delta_q_ / vel_max_ / (1 - accel_ratio_);  // Calculate the required time based on target position and peak speed
  
  // Time partitioning
  tf_acc_ = total_time_ * accel_ratio_; // Acceleration time
  tf_const_ = total_time_ - 2 * tf_acc_; // Constant speed time

  // Calculate constant acceleration
  accel_constant_ = vel_max_ / tf_acc_;

  mc_rtc::log::info("[CollisionBenchmarkController_FirstJointRotation] q_min_ {}, q_max_ {}, delta_q_ {}, vel_max_ {}, total_time_ {}, tf_acc_ {}, tf_const_ {}, accel_constant_ {}",
                    q_min_, q_max_, delta_q_, vel_max_, total_time_, tf_acc_, tf_const_, accel_constant_);
}

bool CollisionBenchmarkController_FirstJointRotation::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);

  if(ctl.datastore().get<bool>("Obstacle detected"))
  {
    ctl.postureTask->reset();
    ctl.postureTask->stiffness(500);
    output("OK");
    return true;
  }

  computeTrapezoidVelocity(ctl);
  ctl.postureTask->refAccel(accel_);
  ctl.postureTask->refVel(velocity_);
  ctl.postureTask->target(q_d_);

  return false;
}

void CollisionBenchmarkController_FirstJointRotation::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

void CollisionBenchmarkController_FirstJointRotation::computeTrapezoidVelocity(mc_control::fsm::Controller & ctl)
{

  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());

  time_counter_ += ctl.timeStep;
  double a_t;
  double v_t;
  double q_zero;

  if(sign_)
  {
    q_zero = q_min_;
  }
  else
  {
    q_zero = q_max_;
  }

  if(time_counter_ <= tf_acc_)
  {
    a_t = accel_constant_ * (sign_ ? 1.0 : -1.0);
    v_t = accel_constant_ * time_counter_ * (sign_ ? 1.0 : -1.0);
    q_d_zero_ = q_zero + 0.5 * accel_constant_ * time_counter_ * time_counter_ * (sign_ ? 1.0 : -1.0);
  }
  else if(time_counter_ > tf_acc_ && time_counter_ <= tf_const_ + tf_acc_)
  {
    a_t = 0.0;
    v_t = vel_max_* (sign_ ? 1.0 : -1.0);
    q_d_zero_ = q_zero + 0.5 * accel_constant_ * tf_acc_ * tf_acc_ * (sign_ ? 1.0 : -1.0) + vel_max_ * (time_counter_ - tf_acc_) * (sign_ ? 1.0 : -1.0);
  }
  else if(time_counter_ > tf_const_ + tf_acc_)
  {
    double t_dec = time_counter_ - (tf_acc_ + tf_const_);
    a_t = -accel_constant_ * (sign_ ? 1.0 : -1.0);
    v_t = vel_max_ * (sign_ ? 1.0 : -1.0) - accel_constant_ * t_dec * (sign_ ? 1.0 : -1.0);
    q_d_zero_ = q_zero + 0.5 * accel_constant_ * tf_acc_ * tf_acc_ * (sign_ ? 1.0 : -1.0) +
     vel_max_ * tf_const_ * (sign_ ? 1.0 : -1.0) + 
     vel_max_ * t_dec * (sign_ ? 1.0 : -1.0) - 0.5 * accel_constant_ * t_dec * t_dec * (sign_ ? 1.0 : -1.0);

    if(time_counter_ >= total_time_)
    {
      time_counter_ = 0.0;
      if(sign_)
      { 
        q_d_zero_ = q_max_;
        sign_ = false;
      }
      else
      {
        q_d_zero_ = q_min_;
        sign_ = true;
      }

    }
  }
  else
  {
    mc_rtc::log::warning(
      "[CollisionBenchmarkController_FirstJointRotation] Out of time");
  }
  
  auto it = q_d_.begin();
  q_d_[it->first][0] = q_d_zero_;
  velocity_[0] = v_t;
  accel_[0] = a_t;
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_FirstJointRotation", CollisionBenchmarkController_FirstJointRotation)
