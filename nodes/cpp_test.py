#!/usr/bin/env python3

import rclpy
from moveit_test.cpp_interface import CppInterface
from time import sleep
import subprocess

def main(args = None):
    rclpy.init(args = args)
    franka_interface = CppInterface("franka")
    fanuc_interface = CppInterface("fanuc")
    
    while True:
        commands = [f"ros2 service call /move_{robot}_arm_home std_srvs/srv/Trigger" for robot in ["fanuc","franka", "motoman", "ur"]]
        subprocesses = [subprocess.Popen(command, shell=True) for command in commands]
        end_codes = [s.wait() for s in subprocesses]
        commands = [f"ros2 service call /move_{robot}_arm_test_state std_srvs/srv/Trigger" for robot in ["fanuc","franka", "motoman", "ur"]]
        subprocesses = [subprocess.Popen(command, shell=True) for command in commands]
        end_codes = [s.wait() for s in subprocesses]
    
if __name__ == "__main__":
    main()