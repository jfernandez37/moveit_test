#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class RobotCommander : public rclcpp::Node
{
public:
  RobotCommander(rclcpp::NodeOptions, moveit::planning_interface::MoveGroupInterface::Options, std::string);
  ~RobotCommander();

private:
  // MoveIt Interfaces 
  moveit::planning_interface::MoveGroupInterface arm_planning_interface_;

  // ROS Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_move_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_move_test_state_srv_;

  // // Service Callbacks
  void ArmMoveHome(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void ArmMoveTestState(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
};