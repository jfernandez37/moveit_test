from moveit.planning import MoveItPy
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Duration, Time
from rclpy.qos import qos_profile_sensor_data

import math
import time

from aprs_interfaces.srv import PickPart

from geometry_msgs.msg import Pose

from moveit_test.utils import build_pose, multiply_pose, rpy_from_quaternion, quaternion_from_euler

from ariac_msgs.msg import (
    Part as PartMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    PartPose as PartPoseMsg
)

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
        
        self.pick_part_clients_ = {robot:self.create_client(PickPart, f"/{robot}_pick_part") for robot in ["fanuc", "franka", "motoman", "ur"]}
        
        self.advanced_logical_camera_sub = self.create_subscription(AdvancedLogicalCameraImageMsg,
                                                                    "/advanced_logical_camera_ros_topic",
                                                                    self.advanced_logical_camera_cb,
                                                                    qos_profile_sensor_data)
        
    def advanced_logical_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
        self.camera_parts = msg.part_poses
        self.camera_pose = msg.sensor_pose
    
    def closest_robot_to_world_pose(self, x: float, y:float):
        return sorted([(robot, abs(math.sqrt((x-coord[0])**2 + (y-coord[1])**2))) for robot, coord in CompetitionInterface._robot_world_coords.items()], key=lambda x: x[1])[0][0]
    
    def pick_part(self, part_to_pick: PartMsg):
        part_pose = Pose()
        found_part = False
        self.log_(f"length of camera parts: {len(self.camera_parts)}")
        for part in self.camera_parts:
            self.log_(f"{part.part.type}=={part_to_pick.type} and {part.part.color} == {part_to_pick.color}")
            part : PartPoseMsg
            if part.part.type == part_to_pick.type and part.part.color == part_to_pick.color:
                part_pose = multiply_pose(self.camera_pose, part.pose)
                found_part = True
                break
        
        closest_robot_to_part = self.closest_robot_to_world_pose(part_pose.position.x, part_pose.position.y)
        
        self.log_("Closest robot to part is " + closest_robot_to_part)
        
        if not found_part:
            self.log_("ERROR: Unable to locate part")
            return False
        self.log_("Part located. Z value = "+str(part_pose.position.z))
        
        request = PickPart.Request()
        request.part = part_to_pick
        request.pose = part_pose
        
        future = self.pick_part_clients_[closest_robot_to_part].call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=150)

        if not future.done():
            raise Error("Timeout reached when calling pick part service")
        
        
    def log_(self, msg: str):
        self.get_logger().info(msg)