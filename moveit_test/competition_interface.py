from moveit.planning import MoveItPy
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Duration, Time

from moveit.planning import PlanningComponent, PlanningSceneMonitor
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from trajectory_msgs.msg import JointTrajectoryPoint

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from moveit.core.robot_state import robotStateToRobotStateMsg, RobotState
from moveit.core.robot_trajectory import RobotTrajectory

from std_msgs.msg import Header

from geometry_msgs.msg import Pose

from moveit_test.utils import build_pose

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

    def __init__(self):
        super().__init__('competition_interface')

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )


        # Moveit_py variables
        self._aprs_robots = MoveItPy(node_name="aprs_robots_moveit_py")

        self._ur_robot : PlanningComponent = self._aprs_robots.get_planning_component("aprs_ur")
        self._fanuc_robot : PlanningComponent = self._aprs_robots.get_planning_component("aprs_fanuc")
        self._franka_robot : PlanningComponent = self._aprs_robots.get_planning_component("aprs_franka")
        self._motoman_robot : PlanningComponent = self._aprs_robots.get_planning_component("aprs_motoman")
        self._robot_info = {
            "ur":{"planning_component":self._ur_robot,
                  "end_link": "tool0",
                  "group_name": "aprs_ur"},
            "fanuc":{"planning_component":self._fanuc_robot,
                  "end_link": "link_6",
                  "group_name": "aprs_fanuc"},
            "franka":{"planning_component":self._franka_robot,
                  "end_link": "fr3_hand_tcp",
                  "group_name": "aprs_franka"},
            "motoman":{"planning_component":self._motoman_robot,
                  "end_link": "link_t",
                  "group_name": "aprs_motoman"}
        }
        self._planning_scene_monitor : PlanningSceneMonitor = self._aprs_robots.get_planning_scene_monitor()
        
        self.get_cartesian_path_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
    
    def _call_get_cartesian_path(self, waypoints : list, 
                                  max_velocity_scaling_factor : float, 
                                  max_acceleration_scaling_factor : float,
                                  avoid_collision : bool,
                                  robot : str = "ur"):

        self.get_logger().info("Getting cartesian path")

        request = GetCartesianPath.Request()

        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()

        request.header = header
        with self._planning_scene_monitor.read_write() as scene:
            request.start_state = robotStateToRobotStateMsg(scene.current_state)

        request.group_name = self._robot_info[robot]["group_name"]
        request.link_name = self._robot_info[robot]["end_link"]
        
        request.waypoints = waypoints
        request.max_step = 0.1
        request.avoid_collisions = avoid_collision
        request.max_velocity_scaling_factor = max_velocity_scaling_factor
        request.max_acceleration_scaling_factor = max_acceleration_scaling_factor

        future = self.get_cartesian_path_client.call_async(request)

        while not future.done():
            pass

        result: GetCartesianPath.Response
        result = future.result()

        if result.fraction < 0.9:
            self.get_logger().error("Unable to plan cartesian trajectory")

        self.get_logger().info("Returning cartesian path")
        return result.solution
    
    def _move_robot_cartesian(self, waypoints, velocity, acceleration, avoid_collision = True, robot = "ur"):
        trajectory_msg = self._call_get_cartesian_path(waypoints, velocity, acceleration, avoid_collision, robot)
        with self._planning_scene_monitor.read_write() as scene:

            trajectory = RobotTrajectory(self._aprs_robots.get_robot_model())
            trajectory.set_robot_trajectory_msg(scene.current_state, trajectory_msg)
            trajectory.joint_model_group_name = self._robot_info[robot]["group_name"]

            trajectory_msg: RobotTrajectoryMsg
            point : JointTrajectoryPoint
            point = trajectory_msg.joint_trajectory.points[-1]
            dur = Duration(seconds=point.time_from_start.sec, nanoseconds=point.time_from_start.nanosec)

            self.get_logger().info(f"Motion will take {dur.nanoseconds} nanoseconds to complete")

        self._aprs_robots.execute(trajectory, controllers=[])
    
    def _plan_and_execute(
        self,
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()
        # execute the plan
        if plan_result:
            logger.info("Executing plan")
            with self._planning_scene_monitor.read_write() as scene:
                robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
        else:
            logger.error("Planning failed")
            return False
        return True
        
    def small_movement(self, robot="ur"):
        with self._planning_scene_monitor.read_write() as scene:
            current_pose = scene.current_state.get_pose(self._robot_info[robot]["end_link"])
        self.print_pose(current_pose)
        current_pose : Pose
        goal_pose = build_pose(current_pose.position.x, current_pose.position.y,
                               current_pose.position.z + 0.1, current_pose.orientation)
        self.print_pose(goal_pose)
        self._move_robot_cartesian([current_pose,goal_pose], 0.4, 0.4, False,robot)
    
    def print_pose(self, pose : Pose):
        self.get_logger().info(f"x: {pose.position.x}\ty: {pose.position.y}\tz: {pose.position.z}\t")
    
    def move_robot_named_position(self, position_name : str, robot : str="ur"):
        
        # Set the start state and goal state for the robot
        with self._planning_scene_monitor.read_write() as scene:
            self._robot_info[robot]["planning_component"].set_start_state(robot_state = scene.current_state)
            self._robot_info[robot]["planning_component"].set_goal_state(configuration_name=position_name)

        self._plan_and_execute(self._aprs_robots,self._robot_info[robot]["planning_component"], self.get_logger())