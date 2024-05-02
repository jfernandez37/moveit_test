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

    urdf = os.path.join(get_package_share_directory("aprs_description"), "urdf/aprs_ur.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("aprs_ur", package_name="aprs_ur_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/aprs_lab_robots.srdf")
        .trajectory_execution(file_path="config/controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("aprs_ur_moveit_config")
            + "/config/moveitpy_config.yaml"
        )
        .to_moveit_configs()
    )

    moveit_py_test = Node(
        package="moveit_test",
        executable="individual_moveit_test_node.py",
        namespace="ur",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time" : True}
        ],
    )
    
    nodes_to_start = [
        moveit_py_test
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])