#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <geometry_msgs/msg/pose.hpp>

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>

class RobotCommander : public rclcpp::Node
{
public:
  RobotCommander(rclcpp::NodeOptions, moveit::planning_interface::MoveGroupInterface::Options, std::string);
  ~RobotCommander();

  void MoveUp();

private:
  // MoveIt Interfaces 
  moveit::planning_interface::MoveGroupInterface arm_planning_interface_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;

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
};