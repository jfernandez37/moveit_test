from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent, PlanningSceneMonitor

from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_state import RobotState

from geometry_msgs.msg import Pose

import os

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


class RobotInterface(Node):
    def __init__(self, robot_prefix: str):
        super().__init__(f"{robot_prefix}_robot_interface")
        
        urdf = os.path.join(get_package_share_directory("aprs_description"), f"urdf/aprs_{robot_prefix}.urdf.xacro")
        
        moveit_config = (
        MoveItConfigsBuilder(f"aprs_{robot_prefix}", package_name=f"aprs_{robot_prefix}_moveit_config")
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path=f"config/aprs_{robot_prefix}.srdf")
        .trajectory_execution(file_path="config/controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory(f"aprs_{robot_prefix}_moveit_config")
            + "/config/moveitpy_config.yaml"
        )
        .to_moveit_configs()
    )
                
        self._robot = MoveItPy(node_name=f"{robot_prefix}_moveit_py", config_dict=moveit_config.to_dict())
        
        self._planning_group: PlanningComponent = self._robot.get_planning_component(f"{robot_prefix}_arm")
        
        self._planning_scene_monitor: PlanningSceneMonitor = self._robot.get_planning_scene_monitor()
        
        self.group_name = f"{robot_prefix}_arm"
        
        end_links = {"fanuc":"link_6",
                     "franka":"fr3_hand_tcp",
                     "motoman":"link_t",
                     "ur":"tool0"}
        
        self.end_link = end_links[robot_prefix]
        
    def get_pose(self):
        with self._planning_scene_monitor.read_only() as scene:
            scene: PlanningScene
            current_state: RobotState = scene.current_state
            
            self.get_logger().info(",".join([str(val) for val in current_state.get_joint_group_positions(self.group_name)]))
            
            return current_state.get_pose(self.end_link)
    
    def print_pose(self, pose: Pose):
        self.get_logger().info(f"x: {pose.position.x}\ty: {pose.position.y}\tz: {pose.position.z}\t")