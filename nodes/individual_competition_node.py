#!/usr/bin/env python3

import threading
import rclpy
from moveit_test.individual_competition_interface import CompetitionInterface
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
    sleep(10)
    # part_to_pick = Part()
    # part_to_pick.type = Part.BATTERY
    # part_to_pick.color = Part.BLUE
    # interface.pick_part(part_to_pick)
    # sleep(10)
    # interface.pick_part(part_to_pick)
    part_to_pick = Part()
    part_to_pick.type = Part.REGULATOR
    part_to_pick.color = Part.GREEN
    interface.pick_part(part_to_pick)
    
    interface.enable_conveyor(True)
    interface.set_conveyor_state(1, 0.5)
    sleep(30)
    interface.enable_conveyor(False)
    
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
