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
    moveit_nodes = []
    for robot_prefix in ["fanuc"]:
        urdf = os.path.join(get_package_share_directory("aprs_description"), f"urdf/aprs_{robot_prefix}.urdf.xacro")
            
            # launch_params_file_path = os.path.join(get_package_share_directory("moveit_test"),"config",f"{robot_prefix}_launch_params.yaml")
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
        print(param_dict.keys())
        moveit_py_test = Node(
            package="moveit_test",
            executable="moveit_test_node.py",
            output="screen",
            parameters=[
                parameters_dict
            ],
        )
        
        moveit_nodes.append(Node(
            package="moveit_test",
            executable="individual_moveit_test_node.py",
            output="screen",
            parameters=[
                # param_dict,
                {"use_sim_time" : True}
            ],
        ))
    
    nodes_to_start = [
        *moveit_nodes
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])