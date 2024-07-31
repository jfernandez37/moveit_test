#include <robot_commander/robot_commander.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char *argv[])
{
  // if (!rclcpp::ok())
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto motoman_commander = std::make_shared<RobotCommander>();

  executor.add_node(motoman_commander);

  std::thread([&executor]() { executor.spin(); }).detach();

  sleep(10);

  motoman_commander->StartTrajectoryMode();

  geometry_msgs::msg::PoseStamped initial_ee_pose = motoman_commander->planning_interface_.getCurrentPose();

  RCLCPP_INFO_STREAM(motoman_commander->get_logger(), 
    "x: " << initial_ee_pose.pose.position.x << " " << 
    "y: " << initial_ee_pose.pose.position.y << " " << 
    "z: " << initial_ee_pose.pose.position.z << " "
  );

  geometry_msgs::msg::Pose target_pose = motoman_commander->BuildPose(
    initial_ee_pose.pose.position.x, 
    initial_ee_pose.pose.position.y,
    initial_ee_pose.pose.position.z - 0.01,
    initial_ee_pose.pose.orientation
  );

  motoman_commander->planning_interface_.setStartStateToCurrentState();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  motoman_commander->MoveRobotCartesian(waypoints, 0.1, 0.1, true);

  sleep(5);

  waypoints.clear();
  waypoints.push_back(initial_ee_pose.pose);
  motoman_commander->MoveRobotCartesian(waypoints, 0.1, 0.1, true);

  // RCLCPP_INFO(motoman_commander->get_logger(), "Movement finished");

  // sleep(2);

  // motoman_commander->StopTrajectoryMode();

  // RCLCPP_INFO(motoman_commander->get_logger(), "After stop trajectory mode");

  rclcpp::shutdown();

  // std::cout << "After shutdown" << std::endl;

  // motoman_commander->planning_interface_.setPoseTarget(target_pose);

  // moveit::planning_interface::MoveGroupInterface::Plan plan;

  // motoman_commander->planning_interface_.plan(plan);

  // for (auto &point : plan.trajectory_.joint_trajectory.points) {
  //   point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
  //   auto dur = rclcpp::Duration(point.time_from_start.sec, point.time_from_start.nanosec);

  //   // RCLCPP_INFO_STREAM(motoman_commander->get_logger(), dur.seconds());
  //   RCLCPP_INFO_STREAM(motoman_commander->get_logger(), point.time_from_start.sec << " " << point.time_from_start.nanosec);

  // }

  // motoman_commander->planning_interface_.execute(plan.trajectory_);

  // executor.spin();
  // for (auto joint_val: motoman_commander->planning_interface_.getCurrentJointValues()) {
  //   RCLCPP_INFO_STREAM(motoman_commander->get_logger(), joint_val);
  // }

  // RCLCPP_INFO_STREAM(motoman_commander->get_logger(), motoman_commander->planning_interface_.getCurrentPose().pose.position.x);

  // rclcpp::shutdown();

  /*
  Motoman
  */

  
  // std::vector<std::string> motoman_node_arguments;
  // rclcpp::NodeOptions motoman_node_options;
  
  // motoman_node_arguments.clear();
  // motoman_node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  // motoman_node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  // motoman_node_arguments.push_back(argv[argc-1]);
  
  // for (auto arg : motoman_node_arguments){
  //   std::cout << arg << std::endl;
  // }

  // motoman_node_options = rclcpp::NodeOptions();
  // motoman_node_options.arguments(motoman_node_arguments);
  // motoman_node_options.allow_undeclared_parameters(true);
  // motoman_node_options.automatically_declare_parameters_from_overrides(true);
  
  // moveit::planning_interface::MoveGroupInterface::Options motoman_moveit_options("motoman_arm", "robot_description", "motoman");
  
  // std::shared_ptr<RobotCommander> motoman_commander = std::make_shared<RobotCommander>(motoman_node_options, motoman_moveit_options, "motoman");
  
}