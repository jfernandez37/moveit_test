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
    all_params = {}
    # for robot in ["franka", "fanuc", "ur", "motoman"]:
    for robot in ["fanuc", "franka"]:
        all_params[f"{robot}_robot_commander"] = {"ros__parameters" : {}}
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
        
        for k,v in parameters_dict.items():
            all_params[f"{robot}_robot_commander"]["ros__parameters"][k] = v
        
        move_groups.append(Node(
            package="moveit_ros_move_group",
            executable="move_group",
            # name="move_group",
            namespace=robot,
            output="screen",
            remappings=[
                ('/joint_states','joint_states')             
            ],
            parameters=[
                parameters_dict
            ],
        ))
        
    with tempfile.NamedTemporaryFile(delete=False) as tmp:
        temp_param_file = tmp.name
        print(temp_param_file)
        params_string = yaml.dump(all_params,sort_keys=False,Dumper=NoAliasDumper)
        tmp.write(bytes(params_string, 'utf-8'))
    
    robot_commander_node = Node(
            package="moveit_test",
            executable="robot_commander_node",
            output="screen",
            arguments=[temp_param_file]
        )

    return LaunchDescription([
        robot_commander_node,
        *move_groups
        ])