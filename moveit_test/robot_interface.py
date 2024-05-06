from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent, PlanningSceneMonitor

from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_state import RobotState

from geometry_msgs.msg import Pose

class RobotInterface(Node):
    def __init__(self):
        super().__init__("robot_interface")
        
        self._robot = MoveItPy(node_name="ur_moveit_py")
        
        self._planning_group: PlanningComponent = self._robot.get_planning_component("aprs_ur")
        
        self._planning_scene_monitor: PlanningSceneMonitor = self._robot.get_planning_scene_monitor()
        
    def get_pose(self):
        with self._planning_scene_monitor.read_only() as scene:
            scene: PlanningScene
            current_state: RobotState = scene.current_state
            
            print(current_state.get_joint_group_positions("aprs_ur"))
            
            return current_state.get_pose("tool0")
    
    def print_pose(self, pose: Pose):
        self.get_logger().info(f"x: {pose.position.x}\ty: {pose.position.y}\tz: {pose.position.z}\t")