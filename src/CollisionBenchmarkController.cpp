#include "CollisionBenchmarkController.h"
#include <mc_rtc/gui/Checkbox.h>

CollisionBenchmarkController::CollisionBenchmarkController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{

  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.99, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Initialize the tasks
  initTargets();
  eeTask = std::make_shared<mc_tasks::EndEffectorTask>("FT_sensor_mounting", robots(),
                                                                    robot().robotIndex(), 1, 1);
  postureTask = getPostureTask(robot().name());
  postureTask->stiffness(100.0);

  initDatastore();
  initLogger();
  initGUI();
  mc_rtc::log::success("CollisionBenchmarkController init done ");
}

bool CollisionBenchmarkController::run()
{
  return mc_control::fsm::Controller::run();
}

void CollisionBenchmarkController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void CollisionBenchmarkController::initTargets()
{ 
  taskPosHome = Eigen::Vector3d(0.45, 0.0, 0.45);
  taskOriHome = Eigen::Quaterniond(-0.5, 0.5, 0.5, 0.5);
  
  postureHome = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};
  
  // Last joints rotation
  postureUp = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {1.40}},  {"joint_7", {1.57}}};
  postureDown = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {0.60}},  {"joint_7", {1.57}}};
  postureRight = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0.96}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};
  postureLeft = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {-0.96}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};
  
  // First joint rotation
  postureFirstJointRotation = {{"joint_1", {0}}, {"joint_2", {0.8}}, {"joint_3", {3.14}}, {"joint_4", {-1.3}},
                   {"joint_5", {0}}, {"joint_6", {0.55}},  {"joint_7", {1.57}}};
  postureFirstJointRotation_end = {{"joint_1", {1.57}}, {"joint_2", {0.8}}, {"joint_3", {3.14}}, {"joint_4", {-1.3}},
                   {"joint_5", {0}}, {"joint_6", {0.55}},  {"joint_7", {1.57}}};

      
  // Forward
  taskPosForward = Eigen::Vector3d(0.65, 0.0, 0.45);

  // Pick Place
  taskOriPickPlace = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
  taskPosPickPlaceDownRight = Eigen::Vector3d(0.45, 0.0, 0.05);
  taskPosPickPlaceDownLeft = Eigen::Vector3d(0.45, -0.3, 0.05);
  taskPosPickPlaceUpRight = Eigen::Vector3d(0.45, 0.0, 0.50);
  taskPosPickPlaceUpLeft = Eigen::Vector3d(0.45, -0.3, 0.50);
}

void CollisionBenchmarkController::initDatastore()
{
  // Kinova Gen3 datastore
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("TorqueMode", "Custom");

  // ConntrollerDatastore
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });
}

void CollisionBenchmarkController::initLogger()
{
  logger().addLogEntry("EndEffectorVel", [this]() { return robot().bodyVelW("FT_sensor_mounting"); });
}

void CollisionBenchmarkController::initGUI()
{
}