#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <aprs_interfaces/srv/pick_part.hpp>
// #include <aprs_interfaces/srv/spawn_sensor.hpp>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

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
  rclcpp::Service<aprs_interfaces::srv::PickPart>::SharedPtr pick_part_srv_;

  // Subscriptions
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_logical_camera_sub_;
  
  // Sensor poses
  geometry_msgs::msg::Pose advanced_logical_camera_pose_;

  // Service Callbacks
  void ArmMoveHome(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void ArmMoveTestState(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  
  // Competitor CBs
  void pick_part_(const std::shared_ptr<aprs_interfaces::srv::PickPart::Request> request,
                            std::shared_ptr<aprs_interfaces::srv::PickPart::Response> response);

  // Sensor CBs
  void advanced_logical_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

  // Data from sensors
  std::vector<ariac_msgs::msg::PartPose> env_parts_;
  
  // Misc. variables
  bool alc_recieved_data = false;
  std::map<int, double> part_heights_ = {
      {ariac_msgs::msg::Part::BATTERY, 0.04},
      {ariac_msgs::msg::Part::PUMP, 0.12},
      {ariac_msgs::msg::Part::REGULATOR, 0.07},
      {ariac_msgs::msg::Part::SENSOR, 0.07}};
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