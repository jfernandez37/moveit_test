#include <robot_commander/robot_commander.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char *argv[])
{
  // if (!rclcpp::ok())
  rclcpp::init(0, nullptr);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  

  /*
  Fanuc
  */
  std::vector<std::string> fanuc_node_arguments;
  rclcpp::NodeOptions fanuc_node_options;
  
  fanuc_node_arguments.clear();
  fanuc_node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  fanuc_node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  fanuc_node_arguments.push_back(argv[argc-2]);

  fanuc_node_options = rclcpp::NodeOptions();
  fanuc_node_options.arguments(fanuc_node_arguments);
  fanuc_node_options.allow_undeclared_parameters(true);
  fanuc_node_options.automatically_declare_parameters_from_overrides(true);

  moveit::planning_interface::MoveGroupInterface::Options fanuc_moveit_options("fanuc_arm", "robot_description", "fanuc");
  std::shared_ptr<RobotCommander> fanuc_commander = std::make_shared<RobotCommander>(fanuc_node_options, fanuc_moveit_options, "fanuc");

  executor.add_node(fanuc_commander);

  executor.spin();
  rclcpp::shutdown();
}