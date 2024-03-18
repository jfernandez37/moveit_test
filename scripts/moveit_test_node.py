#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials
- ros2 run ariac_tutorials tutorial_10.py
'''
import threading
import rclpy
from moveit_test.competition_interface import CompetitionInterface
from rclpy.executors import MultiThreadedExecutor
from time import sleep

def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    while True:
        interface.move_fanuc_random()
        sleep(10)
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
