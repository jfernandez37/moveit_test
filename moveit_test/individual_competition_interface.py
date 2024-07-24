from moveit.planning import MoveItPy
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Duration, Time
from rclpy.qos import qos_profile_sensor_data

import math
import time

from aprs_interfaces.srv import MoveCartesian, MoveToPose

from geometry_msgs.msg import Pose

from std_srvs.srv import Trigger

from moveit_test.utils import build_pose, multiply_pose, rpy_from_quaternion, quaternion_from_euler

class Error(Exception):
  def __init__(self, value: str):
      self.value = value

  def __str__(self):
      return repr(self.value)
  
class CompetitionInterface(Node):
    '''
    Class for a competition interface node.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    '''
    
    _robot_world_coords = {"fanuc": (-0.5222875, -0.073025),
                           "franka": (-1.3145625, -1.375),
                           "motoman": (0.4953, -0.08255),
                           "ur": (0.7864625, -1.375)}
    
    def __init__(self):
        super().__init__('competition_interface')

        self.camera_parts = []
        self.camera_pose = Pose()
        
        self.move_cartesian_clients_ = {robot:self.create_client(MoveCartesian, f"/{robot}_move_cartesian") for robot in self._robot_world_coords.keys()}
        self.move_robot_to_pose_clients_ = {robot:self.create_client(MoveToPose, f"/{robot}_move_to_pose") for robot in self._robot_world_coords.keys()}
        self.move_robot_up_client = self.create_client(Trigger, "/move_motoman_up")
        self.move_robot_down_client = self.create_client(Trigger, "/move_motoman_down")
        
    def closest_robot_to_world_pose(self, x: float, y:float):
        return sorted([(robot, abs(math.sqrt((x-coord[0])**2 + (y-coord[1])**2))) for robot, coord in CompetitionInterface._robot_world_coords.items()], key=lambda x: x[1])[0][0]
        
    def log_(self, msg: str):
        self.get_logger().info(msg)
        
    def move_motoman_up(self):
        request = Trigger.Request()
        future = self.move_robot_up_client.call_async(request)

        while not future.done():
            pass

        if future.result().success:
            self.get_logger().info('Moved motoman up.')
    
    def move_motoman_down(self):
        
        request = Trigger.Request()
        future = self.move_robot_down_client.call_async(request)

        while not future.done():
            pass

        if future.result().success:
            self.get_logger().info('Moved motoman down.')