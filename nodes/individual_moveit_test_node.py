#!/usr/bin/env python3

import threading
import rclpy
from moveit_test.robot_interface import RobotInterface
from rclpy.executors import MultiThreadedExecutor
from time import sleep

def main(args=None):
    rclpy.init(args=args)
    fanuc_interface = RobotInterface("fanuc")
    franka_interface = RobotInterface("franka")
    executor = MultiThreadedExecutor()
    executor.add_node(fanuc_interface)
    executor.add_node(franka_interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    # sleep(10)
    while True:
        try:
            current_pose = fanuc_interface.get_pose()
            fanuc_interface.print_pose(current_pose)
            sleep(2)
            current_pose = franka_interface.get_pose()
            franka_interface.print_pose(current_pose)
            sleep(2)
        except KeyboardInterrupt:
            break
    
    executor.shutdown()


if __name__ == '__main__':
    main()
