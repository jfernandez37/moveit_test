#include <robot_commander/robot_commander.hpp>

RobotCommander::RobotCommander(rclcpp::NodeOptions node_options, std::string robot_name)
 : Node(robot_name + "_robot_commander", "", node_options),
  arm_planning_interface_(std::shared_ptr<rclcpp::Node>(std::move(this)), robot_name + "_arm")
{
  // Use upper joint velocity and acceleration limits
  arm_planning_interface_.setMaxAccelerationScalingFactor(1.0);
  arm_planning_interface_.setMaxVelocityScalingFactor(1.0);

  // Register services
  arm_move_home_srv_ = create_service<std_srvs::srv::Trigger>(
    "/move_" + robot_name + "_arm_home", 
    std::bind(
      &RobotCommander::ArmMoveHome, this,
      std::placeholders::_1, std::placeholders::_2));
  
  arm_move_test_state_srv_ = create_service<std_srvs::srv::Trigger>(
    "/move_" + robot_name + "_arm_test_state", 
    std::bind(
      &RobotCommander::ArmMoveTestState, this,
      std::placeholders::_1, std::placeholders::_2));
}

RobotCommander::~RobotCommander() 
{
  arm_planning_interface_.~MoveGroupInterface();
}

void RobotCommander::ArmMoveHome(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req; // remove unused parameter warning
  arm_planning_interface_.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(arm_planning_interface_.plan(plan));

  if (success) {
    if (static_cast<bool>(arm_planning_interface_.execute(plan))) {
      res->success = true;
    } else {
      res->success = false;
      res->message = "Trajectory execution failed";
    }
  } else {
    res->message = "Unable to generate trajectory";
    res->success = false;
  }
}

void RobotCommander::ArmMoveTestState(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req; // remove unused parameter warning
  arm_planning_interface_.setNamedTarget("test_state");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(arm_planning_interface_.plan(plan));

  if (success) {
    if (static_cast<bool>(arm_planning_interface_.execute(plan))) {
      res->success = true;
    } else {
      res->success = false;
      res->message = "Trajectory execution failed";
    }
  } else {
    res->message = "Unable to generate trajectory";
    res->success = false;
  }
}