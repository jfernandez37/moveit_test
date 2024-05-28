#!/usr/bin/env python3

import rclpy
from moveit_test.cpp_interface import CppInterface
from time import sleep

def main(args = None):
    rclpy.init(args = args)
    interface = CppInterface()
    
    
    while True:
        interface.move_fanuc_home()
        sleep(5)
        interface.move_fanuc_test_state()
        sleep(5)
    
if __name__ == "__main__":
    main()