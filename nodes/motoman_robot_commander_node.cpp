#include <robot_commander/robot_commander.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

int main()
{
  // if (!rclcpp::ok())
  rclcpp::init(0, nullptr);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  
  for(int i = 0; i < 100; i++){
    std::cout << "MOTOMAN COMMANDER STARTED" << std::endl;
  }

  /*
  Motoman
  */

  
  moveit::planning_interface::MoveGroupInterface::Options motoman_moveit_options("motoman_arm", "robot_description", "motoman");
  
  std::shared_ptr<RobotCommander> motoman_commander = std::make_shared<RobotCommander>("motoman");
  
  executor.add_node(motoman_commander);

  executor.spin();
  rclcpp::shutdown();
}