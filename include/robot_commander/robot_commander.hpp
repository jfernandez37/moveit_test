#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <aprs_interfaces/srv/move_cartesian.hpp>
#include <aprs_interfaces/srv/move_to_pose.hpp>


#include <geometry_msgs/msg/pose.hpp>

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>

class RobotCommander : public rclcpp::Node
{
public:
  RobotCommander(std::string);
  ~RobotCommander();

private:
  // MoveIt Interfaces 
  moveit::planning_interface::MoveGroupInterface arm_planning_interface_;

  // ROS Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_move_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_move_test_state_srv_;
  rclcpp::Service<aprs_interfaces::srv::MoveCartesian>::SharedPtr move_cartesian_srv_;
  rclcpp::Service<aprs_interfaces::srv::MoveToPose>::SharedPtr move_to_pose_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_up_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_down_srv_;

  // Subscriptions
  
  // Sensor poses
  geometry_msgs::msg::Pose advanced_logical_camera_pose_;

  // Service Callbacks
  void ArmMoveHome(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void ArmMoveTestState(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void MoveUp(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  
  void MoveDown(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  
  // Competitor CBs
  void move_cartesian_(const std::shared_ptr<aprs_interfaces::srv::MoveCartesian::Request> request,
                            std::shared_ptr<aprs_interfaces::srv::MoveCartesian::Response> response);
  void move_to_pose_(const std::shared_ptr<aprs_interfaces::srv::MoveToPose::Request> request,
                            std::shared_ptr<aprs_interfaces::srv::MoveToPose::Response> response);


  
  // Misc. variables
  bool alc_recieved_data = false;
  double pick_offset_ = 0.003;
  trajectory_processing::TimeOptimalTrajectoryGeneration totg_;
  std::string robot_name_;

  // Utility functions
  geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose, geometry_msgs::msg::Pose);
  double GetYaw(geometry_msgs::msg::Pose);
  geometry_msgs::msg::Pose BuildPose(double, double, double, geometry_msgs::msg::Quaternion);
  geometry_msgs::msg::Quaternion SetRobotOrientation(double);

  // Robot control functions
  bool MoveRobotCartesian(std::vector<geometry_msgs::msg::Pose>, double, double, bool);
  bool MoveRobotToPose(geometry_msgs::msg::Pose);
};