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
    move_groups = []
    robot_commanders = []
    # Robot Commander Node
    urdf = os.path.join(get_package_share_directory("motoman_description"), "urdf/motoman.urdf.xacro")
        
    moveit_config = (
        MoveItConfigsBuilder("motoman", package_name="motoman_moveit_config")
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path="config/motoman.srdf")
        .trajectory_execution(file_path="config/controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    parameters_dict = moveit_config.to_dict()
    parameters_dict["use_sim_time"] = True
    
    move_groups.append(Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        remappings=[
            ('/joint_states','joint_states')             
        ],
        parameters=[
            parameters_dict
        ],
    ))
    
    robot_commanders.append(Node(
            package="moveit_test",
            executable=f"motoman_robot_commander_node",
            output="screen",
            parameters=[
                parameters_dict
            ]
        ))

    individual_competition_node = Node(
        package="moveit_test",
        executable="individual_competition_node.py",
        output="screen"
    )
    
    return LaunchDescription([
        *robot_commanders,
        *move_groups,
        # individual_competition_node
        ])