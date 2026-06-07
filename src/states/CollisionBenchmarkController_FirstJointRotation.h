#pragma once

#include <mc_control/fsm/State.h>
#include "../CollisionBenchmarkController.h"

struct CollisionBenchmarkController_FirstJointRotation : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void computeTrapezoidVelocity(mc_control::fsm::Controller & ctl);

private:
  int jointNumber;
  bool need_home_ = false;
  Eigen::VectorXd velocity_;
  Eigen::VectorXd accel_;
  double q_min_;
  double q_max_;
  double delta_q_;
  double accel_ratio_ = 0.25; // Ratio of acceleration to maximum velocity
  // double vel_max_ = 1.8; //2.0944; // Maximum velocity
  double vel_max_ = 2.0944;
  std::map<std::string, std::vector<double>> q_d_;
  double q_d_zero_;

  double total_time_;
  double tf_acc_; // Acceleration time
  double tf_const_; // Constant speed time
  double accel_constant_;
  double time_counter_ = 0.0;
  bool sign_ = true;

};
