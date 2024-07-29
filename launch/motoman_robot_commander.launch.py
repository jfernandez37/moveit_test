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
    urdf = os.path.join(get_package_share_directory("motoman_description"), f"urdf/motoman.urdf.xacro")
            
    moveit_config = (
        MoveItConfigsBuilder("motoman", package_name="motoman_moveit_config")
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path="config/motoman.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )    
    
    params = moveit_config.to_dict()
    params["publish_robot_description_semantic"] = True
    
    params["kinematics_solver"] = "kdl_kinematics_plugin/KDLKinematicsPlugin"
    params["kinematics_solver_search_resolution"] = 0.005
    params["Kinematics_solver_timeout"] = 0.005
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            params
        ]
    )   
    
    with tempfile.NamedTemporaryFile(delete=False) as tmp:
        temp_param_file = tmp.name
        print(temp_param_file)
        params_string = yaml.dump(params,sort_keys=False,Dumper=NoAliasDumper)
        tmp.write(bytes(params_string, 'utf-8'))
    
    robot_commander_node = Node(
            package="moveit_test",
            executable="motoman_robot_commander_node",
            output="screen",
            parameters=[
                params
            ],
            arguments=[temp_param_file]
        )

    return LaunchDescription([
        robot_commander_node,
        move_group_node
        ])