from moveit.planning import MoveItPy
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from moveit.planning import PlanningComponent, PlanningSceneMonitor

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from moveit.core.robot_state import RobotState

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

        self._fanuc_robot : PlanningComponent = self._aprs_robots.get_planning_component("aprs_fanuc")
        self._planning_scene_monitor : PlanningSceneMonitor = self._aprs_robots.get_planning_scene_monitor()
    
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
    
    def move_fanuc_random(self):
        robot_model = self._aprs_robots.get_robot_model()
        robot_state = RobotState(robot_model)

        # randomize the robot state
        robot_state.set_to_random_positions()

        # set plan start state to current state
        self._fanuc_robot.set_start_state_to_current_state()

        # set goal state to the initialized robot state
        self.get_logger().info("Set goal state to the initialized robot state")
        with self._planning_scene_monitor.read_write() as scene:
            self._fanuc_robot.set_goal_state(robot_state=scene.current_state)

        # plan to goal
        self._plan_and_execute(self._aprs_robots, self._fanuc_robot, self.get_logger(), sleep_time=3.0)
    
    def move_fanuc_random(self):
        robot_model = self._aprs_robots.get_robot_model()
        robot_state = RobotState(robot_model)

        # randomize the robot state
        robot_state.set_to_random_positions()

        # set plan start state to current state
        self._fanuc_robot.set_start_state_to_current_state()

        # set goal state to the initialized robot state
        self.get_logger().info("Set goal state to the initialized robot state")
        self._fanuc_robot.set_goal_state(robot_state=robot_state)

        # plan to goal
        self._plan_and_execute(self._aprs_robots,self._fanuc_robot, self.get_logger())