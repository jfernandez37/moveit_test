import os
from pytest import param
import yaml
import tempfile

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

class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

def generate_launch_description():
    # Robot Commander Node
    urdf = os.path.join(get_package_share_directory("aprs_description"), f"urdf/aprs_lab_robots.urdf.xacro")
            
    moveit_config = (
        MoveItConfigsBuilder(f"aprs_robots", package_name=f"aprs_moveit_config")
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path=f"config/aprs_lab_robots.srdf")
        .trajectory_execution(file_path="config/controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory(f"aprs_moveit_config")
            + "/config/moveitpy_config.yaml"
        )
        .to_moveit_configs()
    )
    
    parameters_dict = moveit_config.to_dict()
    parameters_dict["use_sim_time"] = True
    
    
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        remappings=[
            ('/joint_states','joint_states')             
        ],
        parameters=[
            parameters_dict
        ],
    )
    
    robot_commander_node = Node(
            package="moveit_test",
            executable="moveit_test_node.py",
            output="screen",
            parameters=[
                parameters_dict
            ],
        )

    return LaunchDescription([
        robot_commander_node,
        move_group
        ])