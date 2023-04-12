#!/usr/bin/env python3

"""
Node to Subscribe to Break Beam Sensor
"""

import time
import rclpy
from typing import List
from rclpy.node import Node 
from rclpy.qos import qos_profile_sensor_data
from ariac_msgs.msg import BreakBeamStatus 

from sensor_msgs.msg import LaserScan


class LaserProfilerSubscriber(Node):
    
    def __init__(self):
        super().__init__('break_beam_sensor')
        
        subscription_topic_laser_profiler_conveyor = "/ariac/sensors/laser_profiler_0/scan"

        # Subscriber to the conveyor laser profiler status topic
        self.laser_profiler_conveyor = self.create_subscription(
            LaserScan,
            subscription_topic_laser_profiler_conveyor,
            self.laser_profiler_conveyor_callback,
            qos_profile_sensor_data
        )
        
        self.laser_profiler_conveyor                      # prevent unused variable warnings

    def laser_profiler_conveyor_callback(self, msg: LaserScan):
        '''
        Callback for the breakbeam_conveyor topic 
        Tracks the number of parts that cross the break beam
        '''
        
        self.get_logger().info(" ================================================ ")             
        self.get_logger().info(" Scan message: " + str(msg))             
        time.sleep(10)
        
            
def main(args=None):
    rclpy.init(args=args)
    laser_profiler_subscriber = LaserProfilerSubscriber()
    rclpy.spin(laser_profiler_subscriber)
    laser_profiler_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
