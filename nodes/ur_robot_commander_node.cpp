#include <robot_commander/robot_commander.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char *argv[])
{
  // if (!rclcpp::ok())
  rclcpp::init(0, nullptr);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  
  /*
  UR
  */

  std::vector<std::string> ur_node_arguments;
  rclcpp::NodeOptions ur_node_options;
  
  ur_node_arguments.clear();
  ur_node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  ur_node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  ur_node_arguments.push_back(argv[argc-2]);
  
  ur_node_options = rclcpp::NodeOptions();
  ur_node_options.arguments(ur_node_arguments);
  ur_node_options.allow_undeclared_parameters(true);
  ur_node_options.automatically_declare_parameters_from_overrides(true);
  
  moveit::planning_interface::MoveGroupInterface::Options ur_moveit_options("ur_arm", "robot_description", "ur");
  
  std::shared_ptr<RobotCommander> ur_commander = std::make_shared<RobotCommander>(ur_node_options, ur_moveit_options, "ur");
  
  executor.add_node(ur_commander);

  // std::thread([&executor]() { executor.spin(); }).detach();
  executor.spin();
  rclcpp::shutdown();
}