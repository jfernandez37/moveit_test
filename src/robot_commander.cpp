#include <robot_commander/robot_commander.hpp>

RobotCommander::RobotCommander(rclcpp::NodeOptions node_options, moveit::planning_interface::MoveGroupInterface::Options moveit_options, std::string robot_name)
 : Node(robot_name + "_robot_commander", "", node_options),
  arm_planning_interface_(std::shared_ptr<rclcpp::Node>(std::move(this)), moveit_options, std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(5))
{
  // Use upper joint velocity and acceleration limits
  arm_planning_interface_.setMaxAccelerationScalingFactor(1.0);
  arm_planning_interface_.setMaxVelocityScalingFactor(1.0);

  position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

  robot_name_ = robot_name;
  std::cout << "End of robot commander constructor" << std::endl;
}

RobotCommander::~RobotCommander() 
{
  arm_planning_interface_.~MoveGroupInterface();
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
  robot_trajectory::RobotTrajectory rt(arm_planning_interface_.getCurrentState()->getRobotModel(), robot_name_ + "_arm");
  rt.setRobotTrajectoryMsg(*arm_planning_interface_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  std_msgs::msg::Float64MultiArray target_position;
  std::vector<double> pos(10,0);
  for(auto point : trajectory.joint_trajectory.points){
    for(int i = 0; i < 7; i++){
      pos[i] = point.positions[i];
    }
    target_position.data = pos;
    position_publisher_->publish(target_position);
  }

  return true;
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

void RobotCommander::MoveUp()
{
  arm_planning_interface_.startStateMonitor(10);
  geometry_msgs::msg::PoseStamped initial_ee_pose = arm_planning_interface_.getCurrentPose();

  RCLCPP_INFO(get_logger(), "INSIDE MOVEUP");
  geometry_msgs::msg::Pose target_pose = BuildPose(initial_ee_pose.pose.position.x, initial_ee_pose.pose.position.x, initial_ee_pose.pose.position.z + 0.01,
                                                   initial_ee_pose.pose.orientation);

  std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};

  MoveRobotCartesian(waypoints, 0.3, 0.3, true); 
}

// int main(int argc, char *argv[]){
//   rclcpp::init(argc, argv);

//   auto robot_commander_node = std::make_shared<RobotCommander>();

//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(robot_commander_node);

//   executor.spin();

//   rclcpp::shutdown();
// }