#pragma once
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/EndEffectorTask.h>

#include "api.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>

struct CollisionBenchmarkController_DLLAPI CollisionBenchmarkController : public mc_control::fsm::Controller
{
  CollisionBenchmarkController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

  // Joint Targets
  std::map<std::string, std::vector<double>> postureHome;
  // Last joints rotation
  std::map<std::string, std::vector<double>> postureUp;
  std::map<std::string, std::vector<double>> postureDown;
  std::map<std::string, std::vector<double>> postureRight;
  std::map<std::string, std::vector<double>> postureLeft;
  // First joint rotation
  std::map<std::string, std::vector<double>> postureFirstJointRotation;
  std::map<std::string, std::vector<double>> postureFirstJointRotation_end;

  // EE Targets
  Eigen::Vector3d taskPosHome;
  Eigen::Quaterniond taskOriHome;
  // Forward
  Eigen::Vector3d taskPosForward;
  // Pick Place
  Eigen::Vector3d taskPosPickPlaceDownRight;
  Eigen::Vector3d taskPosPickPlaceDownLeft;
  Eigen::Vector3d taskPosPickPlaceUpRight;
  Eigen::Vector3d taskPosPickPlaceUpLeft;
  Eigen::Quaterniond taskOriPickPlace;

  // Tasks
  std::shared_ptr<mc_tasks::PostureTask> postureTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> eeTask;

private:
  mc_rtc::Configuration config_;
  void initTargets();
  void initDatastore();
  void initLogger();
  void initGUI();
};
