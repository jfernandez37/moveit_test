#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class RobotCommander : public rclcpp::Node
{
public:
  RobotCommander(rclcpp::NodeOptions, std::string);
  ~RobotCommander();

private:
  // MoveIt Interfaces 
  moveit::planning_interface::MoveGroupInterface arm_planning_interface_;

  // ROS Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_move_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_move_test_state_srv_;

  // Service Callbacks
  void ArmMoveHome(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void ArmMoveTestState(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
};

RobotCommander::RobotCommander(rclcpp::NodeOptions node_options, std::string robot_name)
 : Node("robot_commander"),
  arm_planning_interface_(rclcpp::Node::make_shared(robot_name + "_arm", "", node_options), robot_name + "_arm")
{
  // Use upper joint velocity and acceleration limits
  arm_planning_interface_.setMaxAccelerationScalingFactor(1.0);
  arm_planning_interface_.setMaxVelocityScalingFactor(1.0);

  // Register services
  arm_move_home_srv_ = create_service<std_srvs::srv::Trigger>(
    "/robot_commander/move_" + robot_name + "_arm_home", 
    std::bind(
      &RobotCommander::ArmMoveHome, this,
      std::placeholders::_1, std::placeholders::_2));
  
  arm_move_test_state_srv_ = create_service<std_srvs::srv::Trigger>(
    "/robot_commander/move_" + robot_name + "_arm_test_state", 
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

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::vector<std::string> node_arguments;
  node_arguments.push_back("-c fanuc_move_group");
  rclcpp::NodeOptions node_options;
  node_options.arguments(node_arguments);
  /*
  Add a node options that remaps move_group to fanuc_move_group. Make this class have robot_name parameter so we can make one for each robot parameter
  */
  auto robot_commander = std::make_shared<RobotCommander>(node_options, "fanuc");
  rclcpp::spin(robot_commander);
  rclcpp::shutdown();
}