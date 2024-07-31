#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <motoros2_interfaces/srv/start_traj_mode.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <aprs_interfaces/srv/move_cartesian.hpp>
#include <aprs_interfaces/srv/move_to_pose.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>

#include "rclcpp_action/rclcpp_action.hpp"

class RobotCommander : public rclcpp::Node
{
public:
  RobotCommander();
  ~RobotCommander();

  moveit::planning_interface::MoveGroupInterface planning_interface_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  geometry_msgs::msg::Pose BuildPose(double, double, double, geometry_msgs::msg::Quaternion);

  bool MoveRobotCartesian(std::vector<geometry_msgs::msg::Pose>, double, double, bool);
  bool StartTrajectoryMode();
  bool StopTrajectoryMode();
  bool execute_trajectory(moveit_msgs::msg::RobotTrajectory);

private:

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_client_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  
  // MoveIt Interfaces 
  // moveit::planning_interface::MoveGroupInterface planning_interface_;

  // moveit::planning_interface::PlanningSceneInterface planning_scene_;

  // ROS Services
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_move_home_srv_;
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_move_test_state_srv_;
  // rclcpp::Service<aprs_interfaces::srv::MoveCartesian>::SharedPtr move_cartesian_srv_;
  // rclcpp::Service<aprs_interfaces::srv::MoveToPose>::SharedPtr move_to_pose_srv_;
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_up_srv_;
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_down_srv_;

  // Subscriptions
  
  // Sensor poses

  // Service Callbacks
  // void ArmMoveHome(
  //   std_srvs::srv::Trigger::Request::SharedPtr req,
  //   std_srvs::srv::Trigger::Response::SharedPtr res);

  // void ArmMoveTestState(
  //   std_srvs::srv::Trigger::Request::SharedPtr req,
  //   std_srvs::srv::Trigger::Response::SharedPtr res);

  // void MoveUp(
  //   std_srvs::srv::Trigger::Request::SharedPtr req,
  //   std_srvs::srv::Trigger::Response::SharedPtr res);
  
  // void MoveDown(
  //   std_srvs::srv::Trigger::Request::SharedPtr req,
  //   std_srvs::srv::Trigger::Response::SharedPtr res);
  
  // // Competitor CBs
  // void move_cartesian_(const std::shared_ptr<aprs_interfaces::srv::MoveCartesian::Request> request,
  //                           std::shared_ptr<aprs_interfaces::srv::MoveCartesian::Response> response);
  // void move_to_pose_(const std::shared_ptr<aprs_interfaces::srv::MoveToPose::Request> request,
  //                           std::shared_ptr<aprs_interfaces::srv::MoveToPose::Response> response);


  
  // // Misc. variables
  // bool alc_recieved_data = false;
  // double pick_offset_ = 0.003;
  trajectory_processing::TimeOptimalTrajectoryGeneration totg_;
  // std::string robot_name_;

  // // Utility functions
  // geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose, geometry_msgs::msg::Pose);
  // double GetYaw(geometry_msgs::msg::Pose);
  // geometry_msgs::msg::Pose BuildPose(double, double, double, geometry_msgs::msg::Quaternion);
  // geometry_msgs::msg::Quaternion SetRobotOrientation(double);

  // // Robot control functions
  // bool MoveRobotCartesian(std::vector<geometry_msgs::msg::Pose>, double, double, bool);
  // bool MoveRobotToPose(geometry_msgs::msg::Pose);
};