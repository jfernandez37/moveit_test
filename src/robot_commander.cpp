#include <robot_commander/robot_commander.hpp>

RobotCommander::RobotCommander()
 : Node("motoman_robot_commander"),
  planning_interface_(std::shared_ptr<rclcpp::Node>(std::move(this)), "motoman_arm"),
  planning_scene_()
{
  // Use upper joint velocity and acceleration limits
  planning_interface_.setMaxAccelerationScalingFactor(0.1);
  planning_interface_.setMaxVelocityScalingFactor(0.1);

  follow_joint_trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this, "/follow_joint_trajectory");  

  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/trajectories", 10);

  // Register services
  // arm_move_home_srv_ = create_service<std_srvs::srv::Trigger>(
  //   "/move_" + robot_name + "_arm_home", 
  //   std::bind(
  //     &RobotCommander::ArmMoveHome, this,
  //     std::placeholders::_1, std::placeholders::_2));
  
  // arm_move_test_state_srv_ = create_service<std_srvs::srv::Trigger>(
  //   "/move_" + robot_name + "_arm_test_state", 
  //   std::bind(
  //     &RobotCommander::ArmMoveTestState, this,
  //     std::placeholders::_1, std::placeholders::_2));
  
  // move_cartesian_srv_ = create_service<aprs_interfaces::srv::MoveCartesian>(
  //   "/" + robot_name + "_move_cartesian",
  //   std::bind(
  //     &RobotCommander::move_cartesian_, this,
  //     std::placeholders::_1, std::placeholders::_2));
  
  // move_to_pose_srv_ = create_service<aprs_interfaces::srv::MoveToPose>(
  //   "/" + robot_name + "_move_to_pose",
  //   std::bind(
  //     &RobotCommander::move_to_pose_, this,
  //     std::placeholders::_1, std::placeholders::_2));

  // move_up_srv_ = create_service<std_srvs::srv::Trigger>(
  //   "/move_motoman_up",
  //   std::bind(
  //     &RobotCommander::MoveUp, this,
  //     std::placeholders::_1, std::placeholders::_2));
  
  // move_down_srv_ = create_service<std_srvs::srv::Trigger>(
  //   "/move_" + robot_name + "_down",
  //   std::bind(
  //     &RobotCommander::MoveDown, this,
  //     std::placeholders::_1, std::placeholders::_2));

  // robot_name_ = robot_name;
  // std::cout << "End of robot commander constructor" << std::endl;
}

RobotCommander::~RobotCommander() 
{
  planning_interface_.~MoveGroupInterface();
}

bool RobotCommander::StartTrajectoryMode()
{
  // Wait for competition state to be ready

  RCLCPP_INFO_STREAM(get_logger(), "Starting trajectory mode");

  rclcpp::Client<motoros2_interfaces::srv::StartTrajMode>::SharedPtr client = this->create_client<motoros2_interfaces::srv::StartTrajMode>("/start_traj_mode");

  auto request = std::make_shared<motoros2_interfaces::srv::StartTrajMode::Request>();
  auto future = client->async_send_request(request);
  future.wait();

  return true;
}

bool RobotCommander::StopTrajectoryMode(){
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/stop_traj_mode";

  client = this->create_client<std_srvs::srv::Trigger>("/stop_traj_mode");

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  RCLCPP_INFO(get_logger(), "Stopping trajectory mode.");

  auto future = client->async_send_request(request);

  RCLCPP_INFO(get_logger(), "After send request.");

  future.wait();

  RCLCPP_INFO(get_logger(), "Trajectory mode stopped.");

  return future.get()->success;
}
// void RobotCommander::ArmMoveHome(
//   std_srvs::srv::Trigger::Request::SharedPtr req,
//   std_srvs::srv::Trigger::Response::SharedPtr res)
// {
//   RCLCPP_INFO(get_logger(), "Called service");
//   (void)req; // remove unused parameter warning
//   planning_interface_.setNamedTarget("home");

//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   bool success = static_cast<bool>(planning_interface_.plan(plan));

//   if (success) {
//     if (static_cast<bool>(planning_interface_.execute(plan))) {
//       res->success = true;
//     } else {
//       res->success = false;
//       res->message = "Trajectory execution failed";
//     }
//   } else {
//     res->message = "Unable to generate trajectory";
//     res->success = false;
//   }
// }

// void RobotCommander::ArmMoveTestState(
//   std_srvs::srv::Trigger::Request::SharedPtr req,
//   std_srvs::srv::Trigger::Response::SharedPtr res)
// {
//   (void)req; // remove unused parameter warning
//   planning_interface_.setNamedTarget("test_state");

//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   bool success = static_cast<bool>(planning_interface_.plan(plan));

//   if (success) {
//     if (static_cast<bool>(planning_interface_.execute(plan))) {
//       res->success = true;
//     } else {
//       res->success = false;
//       res->message = "Trajectory execution failed";
//     }
//   } else {
//     res->message = "Unable to generate trajectory";
//     res->success = false;
//   }
// }

// void RobotCommander::move_cartesian_(
//   const std::shared_ptr<aprs_interfaces::srv::MoveCartesian::Request> request,
//   std::shared_ptr<aprs_interfaces::srv::MoveCartesian::Response> response
// ){
//   RCLCPP_INFO(this->get_logger(), "Inside move_cartesian cb");
//   std::vector<geometry_msgs::msg::Pose> waypoints;
//   for(auto pose : request->poses){
//     waypoints.push_back(pose);
//   }

//   response->success = MoveRobotCartesian(waypoints, request->asf, request->vsf, request->avoid_collisions);
// }

// void RobotCommander::move_to_pose_(
//   const std::shared_ptr<aprs_interfaces::srv::MoveToPose::Request> request,
//   std::shared_ptr<aprs_interfaces::srv::MoveToPose::Response> response
// ){
//   RCLCPP_INFO(get_logger(), "INSIDE move to pose cb");
//   response->success = MoveRobotToPose(request->pose);
// }

bool RobotCommander::MoveRobotCartesian(
  std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions
){
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = planning_interface_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }
  
  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(planning_interface_.getCurrentState()->getRobotModel(), "motoman_arm");
  rt.setRobotTrajectoryMsg(*planning_interface_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return execute_trajectory(trajectory);
}

// bool RobotCommander::MoveRobotToPose(geometry_msgs::msg::Pose target_pose){
//   planning_interface_.setPoseTarget(target_pose);

//   moveit::planning_interface::MoveGroupInterface::Plan plan;

//   planning_interface_.plan(plan);
//   planning_interface_.move();
//   return true;
// }

// geometry_msgs::msg::Pose RobotCommander::MultiplyPose(
//   geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2
// ){
//   KDL::Frame f1;
//   KDL::Frame f2;

//   tf2::fromMsg(p1, f1);
//   tf2::fromMsg(p2, f2);

//   return tf2::toMsg(f1 * f2);
// }

// double RobotCommander::GetYaw(geometry_msgs::msg::Pose pose){
//   tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
//   tf2::Matrix3x3 m(q);
//   double roll, pitch, yaw;
//   m.getRPY(roll, pitch, yaw);
//   return yaw;
// }

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

// geometry_msgs::msg::Quaternion RobotCommander::SetRobotOrientation(double rotation){
//   tf2::Quaternion tf_q;
//   tf_q.setRPY(0, 3.14159, rotation);

//   geometry_msgs::msg::Quaternion q;

//   q.x = tf_q.x();
//   q.y = tf_q.y();
//   q.z = tf_q.z();
//   q.w = tf_q.w();

//   return q;
// }

// void RobotCommander::MoveUp(
//   std_srvs::srv::Trigger::Request::SharedPtr req,
//   std_srvs::srv::Trigger::Response::SharedPtr res)
// {
//   (void)req;
//   (void)res;
//   // planning_interface_.startStateMonitor(10);
//   geometry_msgs::msg::PoseStamped initial_ee_pose = planning_interface_.getCurrentPose();

//   RCLCPP_INFO(get_logger(), "INSIDE MOVEUP");
//   geometry_msgs::msg::Pose target_pose = BuildPose(initial_ee_pose.pose.position.x, initial_ee_pose.pose.position.y, initial_ee_pose.pose.position.z + 0.01,
//                                                    initial_ee_pose.pose.orientation);

//   std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};

//   MoveRobotCartesian(waypoints, 0.3, 0.3, true); 
// }
  
// void RobotCommander::MoveDown(
//   std_srvs::srv::Trigger::Request::SharedPtr req,
//   std_srvs::srv::Trigger::Response::SharedPtr res)
// {
//   (void)req;
//   (void)res;
//   geometry_msgs::msg::PoseStamped initial_ee_pose = planning_interface_.getCurrentPose();

//   geometry_msgs::msg::Pose target_pose = BuildPose(initial_ee_pose.pose.position.x, initial_ee_pose.pose.position.x, initial_ee_pose.pose.position.z - 0.01,
//                                                    initial_ee_pose.pose.orientation);

//  std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};

//  MoveRobotCartesian(waypoints, 0.3, 0.3, true); 
// }

// int main(int argc, char *argv[]){
//   rclcpp::init(argc, argv);

//   auto robot_commander_node = std::make_shared<RobotCommander>();

//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(robot_commander_node);

//   executor.spin();

//   rclcpp::shutdown();
// }

bool RobotCommander::execute_trajectory(moveit_msgs::msg::RobotTrajectory trajectory){
  control_msgs::action::FollowJointTrajectory::Goal goal_msg;
  goal_msg.trajectory = trajectory.joint_trajectory;

  trajectory_pub_->publish(trajectory.joint_trajectory);

  auto goal_handle_future = follow_joint_trajectory_client_->async_send_goal(goal_msg);
  goal_handle_future.wait();
  return true;
}