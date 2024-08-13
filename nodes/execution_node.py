#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

from moveit_test.execution_interface import MotomanInterface
from time import sleep
import threading

def main(args = None):
    rclpy.init(args = args)
    motoman_interface = MotomanInterface()
    
    executor = MultiThreadedExecutor()
    executor.add_node(motoman_interface)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    
    while(True):
        sleep(0)
    
    motoman_interface.destroy_node()
    rclpy.shutdown()

    
if __name__ == "__main__":
    main()