#!/usr/bin/env python3

import threading
import rclpy
from moveit_test.competition_interface import CompetitionInterface
from rclpy.executors import MultiThreadedExecutor
from time import sleep

from ariac_msgs.msg import Part

def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    # sleep(10)
    # while True:
    #     for robot in ["ur","fanuc", "franka", "motoman"]:
    #         interface.small_movement(robot=robot)
    #     sleep(5)
    part_to_pick = Part()
    part_to_pick.type = Part.BATTERY
    part_to_pick.color = Part.BLUE
    sleep(5)
    interface.pick_part(part_to_pick)
    
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
