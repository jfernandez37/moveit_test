#include <robot_commander/robot_commander.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char *argv[])
{
  // if (!rclcpp::ok())
  rclcpp::init(0, nullptr);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  
  std::vector<std::string> robot_names = {"fanuc"};
  std::vector<std::string> node_arguments;
  rclcpp::NodeOptions node_options;

  node_arguments.clear();
  node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  node_arguments.push_back(argv[argc-1]);
  // node_arguments.push_back(RCL_REMAP_FLAG);
  // node_arguments.push_back("move_group:=fanuc_move_group");
  
  for(auto arg : node_arguments){
    std::cout << arg << std::endl;
  }

  node_options = rclcpp::NodeOptions();
  node_options.arguments(node_arguments);
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  moveit::planning_interface::MoveGroupInterface::Options moveit_options("fanuc_arm", "robot_description", "fanuc");


  std::shared_ptr<RobotCommander> node = std::make_shared<RobotCommander>(node_options, moveit_options, "fanuc");
  executor.add_node(node);

  // std::shared_ptr<RobotCommander> robot_c

  // for(auto robot_name : robot_names){
  //   // node_arguments.clear();
  //   // node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  //   // // node_arguments.push_back(RCL_REMAP_FLAG);
  //   // // node_arguments.push_back("move_group:=" + robot_name + "_move_group");
  //   // node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  //   // node_arguments.push_back(argv[argc-1]);
  //   std::cout << "Robot name: " << robot_name << std::endl;
  //   for(auto arg : node_arguments){
  //     std::cout << arg << std::endl;
  //   }
  //   node_options = rclcpp::NodeOptions();
  //   // node_options.arguments(node_arguments);
  //   node_options.allow_undeclared_parameters(true);
  //   node_options.automatically_declare_parameters_from_overrides(true);

  //   auto node = 

    
  // }
  // std::thread([&executor]() { executor.spin(); }).detach();
  executor.spin();
  rclcpp::shutdown();
}