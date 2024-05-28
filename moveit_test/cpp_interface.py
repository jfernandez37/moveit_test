import rclpy
from rclpy.node import Node


from std_srvs.srv import Trigger

class CppInterface(Node):
    def __init__(self):
        super().__init__("cpp_interface")
        
        # Service client for moving the floor robot to the home position
        self._move_fanuc_home = self.create_client(
            Trigger, '/robot_commander/move_fanuc_arm_home')
        
        # Service client for moving the ceiling robot to the home position
        self._move_fanuc_test_state = self.create_client(
            Trigger, '/robot_commander/move_fanuc_arm_test_state')
    
    def move_fanuc_home(self):
        request = Trigger.Request()

        if not self._move_fanuc_home.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Robot commander node not running')
            return

        future = self._move_fanuc_home.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved Fanuc to home position')
        else:
            self.get_logger().warn(future.result().message)
    
    def move_fanuc_test_state(self):
        request = Trigger.Request()

        if not self._move_fanuc_test_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Robot commander node not running')
            return

        future = self._move_fanuc_test_state.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved Fanuc to test position')
        else:
            self.get_logger().warn(future.result().message)