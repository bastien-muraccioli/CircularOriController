#pragma once
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <mc_tasks/CompliantEndEffectorTask.h>

#include "api.h"
#include <Eigen/src/Core/Matrix.h>

struct CollisionBenchmarkController_DLLAPI CollisionBenchmarkController : public mc_control::fsm::Controller
{
  CollisionBenchmarkController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  // Targets
  std::map<std::string, std::vector<double>> postureHome;
  std::map<std::string, std::vector<double>> postureUp;
  std::map<std::string, std::vector<double>> postureDown;
  std::map<std::string, std::vector<double>> postureRight;
  std::map<std::string, std::vector<double>> postureLeft;
  std::map<std::string, std::vector<double>> postureBigRotate;
  std::map<std::string, std::vector<double>> postureBigRotate_end;
  std::map<std::string, std::vector<double>> postureBigUp;
  std::map<std::string, std::vector<double>> postureBigDown;
  std::map<std::string, std::vector<double>> postureHorizontalStart;
  std::map<std::string, std::vector<double>> postureBigLeft;
  std::map<std::string, std::vector<double>> postureBigRight;

  Eigen::Vector3d taskPosHome;
  Eigen::Vector3d taskPosForward;

  // Tasks
  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;
  std::shared_ptr<mc_tasks::CompliantEndEffectorTask> compEETask;
  std::string reaction_mode = "ReactionSimple";

  // State variables
  bool isTorqueControl = false;
  bool isSuperTwisting = false;
  // bool controlModeRequest = true;

private:
  mc_rtc::Configuration config_;
};
