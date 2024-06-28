#!/usr/bin/env python3

import threading
import rclpy
from moveit_test.robot_interface import RobotInterface
from rclpy.executors import MultiThreadedExecutor
from time import sleep

def main(args=None):
    rclpy.init(args=args)

    fanuc_interface = RobotInterface("fanuc")
    # franka_interface = RobotInterface("franka")
         
    executor = MultiThreadedExecutor()
    executor.add_node(fanuc_interface)
    # executor.add_node(franka_interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    sleep(5)

    while True:
        try:
            fanuc_interface.move_to_named_joint_state("home")
            # franka_interface.move_to_named_joint_state("home")
            sleep(2)
            fanuc_interface.move_to_named_joint_state("test_state")
            # franka_interface.move_to_named_joint_state("test_state")
            sleep(2)
        except KeyboardInterrupt:
            break
    
    executor.shutdown()


if __name__ == '__main__':
    main()
