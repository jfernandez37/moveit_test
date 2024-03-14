import os
from pytest import param
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    urdf = os.path.join(get_package_share_directory("aprs_description"), "urdf/aprs_lab.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("ariac_robots", package_name="aprs_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/aprs_robots.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("moveit_test")
            + "/config/moveit_test_config.yaml"
        )
        .to_moveit_configs()
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    
    parameters_dict = moveit_config.to_dict()
    parameters_dict["use_sim_time"] = True
    parameters_dict.update(trajectory_execution)
    parameters_dict.update(planning_scene_monitor_parameters)
    moveit_py_test = Node(
        package="moveit_test",
        executable="moveit_test_node.py",
        output="screen",
        parameters=[
            parameters_dict
        ],
    )
    
    nodes_to_start = [
        moveit_py_test
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])