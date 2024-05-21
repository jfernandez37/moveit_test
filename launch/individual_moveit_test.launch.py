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
    
    moveit_py_test = Node(
        package="moveit_test",
        executable="individual_moveit_test_node.py",
        output="screen",
        parameters=[
            # param_dict,
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