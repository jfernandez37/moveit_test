#include <robot_commander/robot_commander.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  
  std::vector<std::string> robot_names = {"fanuc"};
  std::vector<std::string> node_arguments;
  rclcpp::NodeOptions node_options;

  for(auto robot_name : robot_names){
    node_arguments.clear();
    // node_arguments.push_back(RCL_ROS_ARGS_FLAG);
    // node_arguments.push_back(RCL_REMAP_FLAG);
    // node_arguments.push_back("move_group:=" + robot_name + "_move_group");
    node_arguments.push_back(RCL_PARAM_FILE_FLAG);
    node_arguments.push_back(argv[argc-2]);
    node_options = rclcpp::NodeOptions();
    node_options.arguments(node_arguments);
    for (auto s : node_options.arguments()){
        std::cout << s << "\t";
    }
    std::cout << std::endl;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    executor.add_node(std::make_shared<RobotCommander>(node_options, robot_name));
  }
  std::thread([&executor]() { executor.spin(); }).detach();
  rclcpp::shutdown();
}