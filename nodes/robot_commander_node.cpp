#include <robot_commander/robot_commander.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char *argv[])
{
  // if (!rclcpp::ok())
  rclcpp::init(0, nullptr);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  
  // std::vector<std::string> robot_names = {"franka"};
  std::vector<std::string> fanuc_node_arguments;
  std::vector<std::string> franka_node_arguments;
  rclcpp::NodeOptions fanuc_node_options;
  rclcpp::NodeOptions franka_node_options;

  fanuc_node_arguments.clear();
  fanuc_node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  fanuc_node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  fanuc_node_arguments.push_back(argv[argc-2]);

  franka_node_arguments.clear();
  franka_node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  franka_node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  franka_node_arguments.push_back(argv[argc-2]);
  // node_arguments.push_back(RCL_REMAP_FLAG);
  // node_arguments.push_back("move_group:=franka_move_group");
  
  for(auto arg : fanuc_node_arguments){
    std::cout << arg << std::endl;
  }

  fanuc_node_options = rclcpp::NodeOptions();
  fanuc_node_options.arguments(fanuc_node_arguments);
  fanuc_node_options.allow_undeclared_parameters(true);
  fanuc_node_options.automatically_declare_parameters_from_overrides(true);

  franka_node_options = rclcpp::NodeOptions();
  franka_node_options.arguments(franka_node_arguments);
  franka_node_options.allow_undeclared_parameters(true);
  franka_node_options.automatically_declare_parameters_from_overrides(true);

  moveit::planning_interface::MoveGroupInterface::Options fanuc_moveit_options("fanuc_arm", "robot_description", "fanuc");
  moveit::planning_interface::MoveGroupInterface::Options franka_moveit_options("franka_arm", "robot_description", "franka");
  
  std::shared_ptr<RobotCommander> franka_commander = std::make_shared<RobotCommander>(franka_node_options, franka_moveit_options, "franka");
  sleep(3);

  std::shared_ptr<RobotCommander> fanuc_commander = std::make_shared<RobotCommander>(fanuc_node_options, fanuc_moveit_options, "fanuc");
  
  
  
  executor.add_node(fanuc_commander);
  executor.add_node(franka_commander);

  // std::thread([&executor]() { executor.spin(); }).detach();
  executor.spin();
  rclcpp::shutdown();
}