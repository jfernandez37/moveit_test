#!/usr/bin/env python3

import rclpy
from moveit_test.cpp_interface import CppInterface
from time import sleep

def main(args = None):
    rclpy.init(args = args)
    franka_interface = CppInterface("franka")
    # fanuc_interface = CppInterface("fanuc")
    
    
    while True:
        # fanuc_interface.move_robot_home()
        franka_interface.move_robot_home()
        sleep(5)
        # fanuc_interface.move_robot_test_state()
        franka_interface.move_robot_test_state
        sleep(5)
    
if __name__ == "__main__":
    main()