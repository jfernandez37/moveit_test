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

def generate_launch_description():
    robot_commanders = []
    move_groups = []
    for robot in ["franka"]:
        # Robot Commander Node
        urdf = os.path.join(get_package_share_directory("aprs_description"), f"urdf/aprs_{robot}.urdf.xacro")
                
                # launch_params_file_path = os.path.join(get_package_share_directory("moveit_test"),"config",f"{robot}_launch_params.yaml")
        moveit_config = (
            MoveItConfigsBuilder(f"aprs_{robot}", package_name=f"aprs_{robot}_moveit_config")
            .robot_description(file_path=urdf)
            .robot_description_semantic(file_path=f"config/aprs_{robot}.srdf")
            .trajectory_execution(file_path="config/controllers.yaml")
            .planning_pipelines(pipelines=["ompl"])
            .joint_limits(file_path="config/joint_limits.yaml")
            .moveit_cpp(
                file_path=get_package_share_directory(f"aprs_{robot}_moveit_config")
                + "/config/moveitpy_config.yaml"
            )
            .to_moveit_configs()
        )
        
        parameters_dict = moveit_config.to_dict()
        parameters_dict["use_sim_time"] = True
        parameters_dict["robot_name"] = robot
        
        print(parameters_dict.keys())
        
        robot_commanders.append(Node(
            package="moveit_test",
            executable="robot_commander",
            output="screen",
            parameters=[parameters_dict]
        ))
        
        move_groups.append(Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name=f"{robot}_move_group",
            # namespace="fanuc",
            output="screen",
            # remappings=[
            #     ('/joint_states', '/fanuc/joint_states')             
            # ],
            parameters=[
                parameters_dict
            ],
        ))

    return LaunchDescription([
        *robot_commanders,
        *move_groups
        ])