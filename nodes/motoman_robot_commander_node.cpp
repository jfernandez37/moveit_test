#include <robot_commander/robot_commander.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char *argv[])
{
  // if (!rclcpp::ok())
  rclcpp::init(0, nullptr);
  
  rclcpp::executors::SingleThreadedExecutor executor;

  /*
  Motoman
  */

  
  std::vector<std::string> motoman_node_arguments;
  rclcpp::NodeOptions motoman_node_options;
  
  motoman_node_arguments.clear();
  motoman_node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  motoman_node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  motoman_node_arguments.push_back(argv[argc-1]);
  
  for (auto arg : motoman_node_arguments){
    std::cout << arg << std::endl;
  }

  motoman_node_options = rclcpp::NodeOptions();
  motoman_node_options.arguments(motoman_node_arguments);
  motoman_node_options.allow_undeclared_parameters(true);
  motoman_node_options.automatically_declare_parameters_from_overrides(true);
  
  moveit::planning_interface::MoveGroupInterface::Options motoman_moveit_options("motoman_arm", "robot_description", "motoman");
  
  std::shared_ptr<RobotCommander> motoman_commander = std::make_shared<RobotCommander>(motoman_node_options, motoman_moveit_options, "motoman");
  executor.add_node(motoman_commander);

  executor.spin();

  std::cout << "Before sleep and move up call" << std::endl;

  sleep(5);
  motoman_commander->MoveUp();
  
  rclcpp::shutdown();
}