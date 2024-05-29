import rclpy
from rclpy.node import Node


from std_srvs.srv import Trigger

class CppInterface(Node):
    def __init__(self, robot_name: str):
        super().__init__("cpp_interface")
        
        self.robot_name = robot_name
        
        # Service client for moving the floor robot to the home position
        self._move_robot_home = self.create_client(
            Trigger, f'/robot_commander/move_{robot_name}_arm_home')
        
        # Service client for moving the ceiling robot to the home position
        self._move_robot_test_state = self.create_client(
            Trigger, f'/robot_commander/move_{robot_name}_arm_test_state')
    
    def move_robot_home(self):
        request = Trigger.Request()

        if not self._move_robot_home.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Robot commander node not running')
            return

        future = self._move_robot_home.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {self.robot_name} to home position')
        else:
            self.get_logger().warn(future.result().message)
    
    def move_robot_test_state(self):
        request = Trigger.Request()

        if not self._move_robot_test_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Robot commander node not running')
            return

        future = self._move_robot_test_state.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {self.robot_name} to test position')
        else:
            self.get_logger().warn(future.result().message)