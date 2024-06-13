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
  
  for(auto arg : fanuc_node_arguments){
    std::cout << arg << std::endl;
  }

  fanuc_node_options = rclcpp::NodeOptions();
  fanuc_node_options.arguments(fanuc_node_arguments);
  fanuc_node_options.allow_undeclared_parameters(true);
  fanuc_node_options.automatically_declare_parameters_from_overrides(true);

  moveit::planning_interface::MoveGroupInterface::Options fanuc_moveit_options("fanuc_arm", "robot_description", "fanuc");
  std::shared_ptr<RobotCommander> fanuc_commander = std::make_shared<RobotCommander>(fanuc_node_options, fanuc_moveit_options, "fanuc");

  executor.add_node(fanuc_commander);
  

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


  // /*
  // Motoman
  // */

  // std::vector<std::string> motoman_node_arguments;
  // rclcpp::NodeOptions motoman_node_options;
  
  // motoman_node_arguments.clear();
  // motoman_node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  // motoman_node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  // motoman_node_arguments.push_back(argv[argc-2]);
  
  // motoman_node_options = rclcpp::NodeOptions();
  // motoman_node_options.arguments(motoman_node_arguments);
  // motoman_node_options.allow_undeclared_parameters(true);
  // motoman_node_options.automatically_declare_parameters_from_overrides(true);
  
  // moveit::planning_interface::MoveGroupInterface::Options motoman_moveit_options("motoman_arm", "robot_description", "motoman");
  
  // std::shared_ptr<RobotCommander> motoman_commander = std::make_shared<RobotCommander>(motoman_node_options, motoman_moveit_options, "motoman");
  
  // executor.add_node(motoman_commander);


  // /*
  // UR
  // */

  // std::vector<std::string> ur_node_arguments;
  // rclcpp::NodeOptions ur_node_options;
  
  // ur_node_arguments.clear();
  // ur_node_arguments.push_back(RCL_ROS_ARGS_FLAG);
  // ur_node_arguments.push_back(RCL_PARAM_FILE_FLAG);
  // ur_node_arguments.push_back(argv[argc-2]);
  
  // ur_node_options = rclcpp::NodeOptions();
  // ur_node_options.arguments(ur_node_arguments);
  // ur_node_options.allow_undeclared_parameters(true);
  // ur_node_options.automatically_declare_parameters_from_overrides(true);
  
  // moveit::planning_interface::MoveGroupInterface::Options ur_moveit_options("ur_arm", "robot_description", "ur");
  
  // std::shared_ptr<RobotCommander> ur_commander = std::make_shared<RobotCommander>(ur_node_options, ur_moveit_options, "ur");
  
  // executor.add_node(ur_commander);

  // std::thread([&executor]() { executor.spin(); }).detach();
  executor.spin();
  rclcpp::shutdown();
}