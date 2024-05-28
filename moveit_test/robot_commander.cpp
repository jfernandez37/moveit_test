#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class RobotCommander : public rclcpp::Node
{
public:
  RobotCommander();
  ~RobotCommander();

private:
  // MoveIt Interfaces 
  moveit::planning_interface::MoveGroupInterface fanuc_arm_;

  // ROS Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fanuc_arm_move_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fanuc_arm_move_test_state_srv_;

  // Service Callbacks
  void FanucArmMoveHome(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void FanucArmMoveTestState(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
};

RobotCommander::RobotCommander()
 : Node("robot_commander"),
  fanuc_arm_(std::shared_ptr<rclcpp::Node>(std::move(this)), "fanuc_arm")
{
  RCLCPP_INFO(get_logger(), "Inside constructor\n");
  // Use upper joint velocity and acceleration limits
  fanuc_arm_.setMaxAccelerationScalingFactor(1.0);
  fanuc_arm_.setMaxVelocityScalingFactor(1.0);

  // Register services
  fanuc_arm_move_home_srv_ = create_service<std_srvs::srv::Trigger>(
    "/robot_commander/move_fanuc_arm_home", 
    std::bind(
      &RobotCommander::FanucArmMoveHome, this,
      std::placeholders::_1, std::placeholders::_2));
  
  fanuc_arm_move_test_state_srv_ = create_service<std_srvs::srv::Trigger>(
    "/robot_commander/move_fanuc_arm_test_state", 
    std::bind(
      &RobotCommander::FanucArmMoveTestState, this,
      std::placeholders::_1, std::placeholders::_2));
}

RobotCommander::~RobotCommander() 
{
  fanuc_arm_.~MoveGroupInterface();
}

void RobotCommander::FanucArmMoveHome(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req; // remove unused parameter warning
  fanuc_arm_.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(fanuc_arm_.plan(plan));

  if (success) {
    if (static_cast<bool>(fanuc_arm_.execute(plan))) {
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

void RobotCommander::FanucArmMoveTestState(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req; // remove unused parameter warning
  fanuc_arm_.setNamedTarget("test_state");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(fanuc_arm_.plan(plan));

  if (success) {
    if (static_cast<bool>(fanuc_arm_.execute(plan))) {
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
  auto robot_commander = std::make_shared<RobotCommander>();
  rclcpp::spin(robot_commander);
  rclcpp::shutdown();
}