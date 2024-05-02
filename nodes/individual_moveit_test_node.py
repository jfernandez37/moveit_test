#!/usr/bin/env python3

import threading
import rclpy
from moveit_test.robot_interface import RobotInterface
from rclpy.executors import MultiThreadedExecutor
from time import sleep

def main(args=None):
    rclpy.init(args=args)
    interface = RobotInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    # sleep(10)
    while True:
        try:
            current_pose = interface.get_pose()
            interface.print_pose(current_pose)
            sleep(2)
        except KeyboardInterrupt:
            break
    
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
