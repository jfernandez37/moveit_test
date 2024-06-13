#include <robot_commander/robot_commander.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char *argv[])
{
  // if (!rclcpp::ok())
  rclcpp::init(0, nullptr);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  
  /*
  Franka
  */

  std::vector<std::string> franka_node_arguments;
  rclcpp::NodeOptions franka_node_options;
  
  franka_node_arguments.clear();
  franka_node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  franka_node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  franka_node_arguments.push_back(argv[argc-2]);
  
  franka_node_options = rclcpp::NodeOptions();
  franka_node_options.arguments(franka_node_arguments);
  franka_node_options.allow_undeclared_parameters(true);
  franka_node_options.automatically_declare_parameters_from_overrides(true);
  
  moveit::planning_interface::MoveGroupInterface::Options franka_moveit_options("franka_arm", "robot_description", "franka");
  
  std::shared_ptr<RobotCommander> franka_commander = std::make_shared<RobotCommander>(franka_node_options, franka_moveit_options, "franka");
  
  executor.add_node(franka_commander);

  executor.spin();
  rclcpp::shutdown();
}