from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent, PlanningSceneMonitor, PlanRequestParameters

from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_state import RobotState
from moveit.core.planning_interface import MotionPlanResponse
from moveit.core.controller_manager import ExecutionStatus

from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes

import os

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

class ItemNotFound(Exception):
    pass

class ExecutionFailure(Exception):
    pass

class PlanningFailure(Exception):
    pass

class RobotInterface(Node):
    def __init__(self, robot_prefix: str):
        super().__init__(f"{robot_prefix}_robot_interface")
        
        urdf = os.path.join(get_package_share_directory("aprs_description"), f"urdf/aprs_{robot_prefix}.urdf.xacro")
        
        # launch_params_file_path = os.path.join(get_package_share_directory("moveit_test"),"config","launch_params.yaml")
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
        
        config_dict = moveit_config.to_dict()
        # config_dict["use_sim_time"] = True
        print(config_dict.keys())
                
        self._robot = MoveItPy(node_name=f"{robot_prefix}_moveit_py", config_dict=config_dict, provide_planning_service=True)
        # self._robot = MoveItPy(node_name=f"{robot_prefix}_moveit_py", config_dict=config_dict)
        
        self._planning_group: PlanningComponent = self._robot.get_planning_component(f"{robot_prefix}_arm")
        
        self._planning_scene_monitor: PlanningSceneMonitor = self._robot.get_planning_scene_monitor()
        
        self.group_name = f"{robot_prefix}_arm"
        
        end_links = {"fanuc":"link_6",
                     "franka":"fr3_hand_tcp",
                     "motoman":"link_t",
                     "ur":"tool0"}
        
        self.end_link = end_links[robot_prefix]
        
    def _plan_and_execute(
        self,
        single_plan_parameters=None,
        multi_plan_parameters=None,
    ):
        """Helper function to plan and execute a motion."""
        single_plan_parameters = PlanRequestParameters(self._robot, self.group_name)
        single_plan_parameters.max_acceleration_scaling_factor = 0.3
        single_plan_parameters.max_velocity_scaling_factor = 0.3
        single_plan_parameters.planning_pipeline = "ompl"
        single_plan_parameters.planner_id = "RRTConnectkConfigDefault"
        
        # plan to goal
        self.get_logger().info("Planning trajectory")
        plan: MotionPlanResponse = self._planning_group.plan(single_plan_parameters=single_plan_parameters)
        plan_result: MoveItErrorCodes = plan.error_code
        
        # execute the plan
        if plan_result.val == MoveItErrorCodes.SUCCESS:
            if not plan.trajectory.apply_totg_time_parameterization(0.3, 0.3):
                self.get_logger().warn('Unable to retime trajectory')
            
            execution: ExecutionStatus =  self._robot.execute(plan.trajectory, controllers=[])
            
            if not execution.status == "SUCCEEDED":
                self.get_logger().error(f'Unable to complete trajectory. Error: {execution.status}')
        else:
            self.get_logger().error(f'Unable to plan trajectory. Error code: {plan_result.val}')
    
    def get_pose(self):
        with self._planning_scene_monitor.read_only() as scene:
            scene: PlanningScene
            current_state: RobotState = scene.current_state
            
            self.get_logger().info(",".join([str(val) for val in current_state.get_joint_group_positions(self.group_name)]))
            
            return current_state.get_pose(self.end_link)
    
    def print_pose(self, pose: Pose):
        self.get_logger().info(f"x: {pose.position.x}\ty: {pose.position.y}\tz: {pose.position.z}\t")
        
    def move_to_named_joint_state(self, joint_state: str):
        with self._planning_scene_monitor.read_write() as scene:
            self._planning_group.set_start_state(robot_state = scene.current_state)
            
        self._planning_group.set_goal_state(configuration_name=joint_state)
        
        self._plan_and_execute()