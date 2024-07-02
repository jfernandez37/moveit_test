#include <robot_commander/robot_commander.hpp>

RobotCommander::RobotCommander(rclcpp::NodeOptions node_options, moveit::planning_interface::MoveGroupInterface::Options moveit_options, std::string robot_name)
 : Node(robot_name + "_robot_commander", "", node_options),
  arm_planning_interface_(std::shared_ptr<rclcpp::Node>(std::move(this)), moveit_options, std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(5))
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

  pick_part_srv_ = create_service<aprs_interfaces::srv::PickPart>(
    "/" + robot_name + "_pick_part",
    std::bind(
      &RobotCommander::pick_part_, this,
      std::placeholders::_1, std::placeholders::_2));

  // Advanced logical camera subscription
  advanced_logical_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
    "/advanced_logical_camera_ros_topic", rclcpp::SensorDataQoS(),
    std::bind(&RobotCommander::advanced_logical_camera_cb, this, std::placeholders::_1)
  );
}

RobotCommander::~RobotCommander() 
{
  arm_planning_interface_.~MoveGroupInterface();
}

void RobotCommander::advanced_logical_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg){
  if(!alc_recieved_data){
    RCLCPP_DEBUG(get_logger(), "Received data from advanced logical camera");
    alc_recieved_data = true;
  }

  env_parts_ = msg->part_poses;
  advanced_logical_camera_pose_ = msg->sensor_pose;
}

void RobotCommander::ArmMoveHome(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Called service");
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

void RobotCommander::pick_part_(
  const std::shared_ptr<aprs_interfaces::srv::PickPart::Request> request,
  std::shared_ptr<aprs_interfaces::srv::PickPart::Response> response
){
  geometry_msgs::msg::Pose part_pose = request->pose;
  ariac_msgs::msg::Part part_to_pick = request->part;

  double part_rotation = GetYaw(part_pose);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));
  
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_, SetRobotOrientation(part_rotation)));

  MoveRobotCartesian(waypoints, 0.3, 0.3, true);

  response->success = true;
}

bool RobotCommander::MoveRobotCartesian(
  std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions
){
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = arm_planning_interface_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(arm_planning_interface_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*arm_planning_interface_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(arm_planning_interface_.execute(trajectory));
}

geometry_msgs::msg::Pose RobotCommander::MultiplyPose(
  geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2
){
  KDL::Frame f1;
  KDL::Frame f2;

  tf2::fromMsg(p1, f1);
  tf2::fromMsg(p2, f2);

  return tf2::toMsg(f1 * f2);
}

double RobotCommander::GetYaw(geometry_msgs::msg::Pose pose){
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Pose RobotCommander::BuildPose(
  double x, double y, double z, geometry_msgs::msg::Quaternion orientation
){
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = orientation;

  return pose;
}

geometry_msgs::msg::Quaternion RobotCommander::SetRobotOrientation(double rotation){
  tf2::Quaternion tf_q;
  tf_q.setRPY(0, 3.14159, rotation);

  geometry_msgs::msg::Quaternion q;

  q.x = tf_q.x();
  q.y = tf_q.y();
  q.z = tf_q.z();
  q.w = tf_q.w();

  return q;
}

// int main(int argc, char *argv[]){
//   rclcpp::init(argc, argv);

//   auto robot_commander_node = std::make_shared<RobotCommander>();

//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(robot_commander_node);

//   executor.spin();

//   rclcpp::shutdown();
// }