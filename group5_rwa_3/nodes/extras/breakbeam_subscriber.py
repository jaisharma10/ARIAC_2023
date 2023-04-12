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

class BreakBeamSubscriber(Node):
    
    def __init__(self):
        super().__init__('break_beam_sensor')
        
        subscription_topic_bb_conveyor = "/ariac/sensors/breakbeam_conveyor/status"

        # Subscriber to the conveyor break beam status topic
        self.break_beam_conveyor = self.create_subscription(
            BreakBeamStatus,
            subscription_topic_bb_conveyor,
            self.breakbeam_conveyor_callback,
            qos_profile_sensor_data
        )
        
        self.break_beam_conveyor                      # prevent unused variable warnings

        self.conveyor_part_count: int = 0             # increment for each part that crosses the beam
        self.object_detected: bool = False            # track if beam is broken
        self.conveyor_speed: float = 0.2              # speed of conveyor (m/s)
        self.pickZone_gap: float = 30.0               # pick zone is selected to be 20 seconds downstream
        self.break_beam_y: float =3.5                 # y position of break beam sensor

        self.conveyor_part_detected_list: List = []   # store occurence of detected part and time
        self.future_part_info: List = []                   # store future pose of detected part and time
        
    def breakbeam_conveyor_callback(self, msg: BreakBeamStatus):
        '''
        Callback for the breakbeam_conveyor topic 
        Tracks the number of parts that cross the break beam
        '''
        
        if msg.object_detected: self.object_detected =  True
        
        if self.object_detected: 
            self.conveyor_part_count += 1
            self.get_logger().info(" ================================================ ")             
            self.get_logger().info(" Conveyor Part Count is: %s"  %  self.conveyor_part_count) 
            self.get_logger().info(" Conveyor Part Time Stamp: %s sec and %s nanosec"  %  (msg.header.stamp.sec,msg.header.stamp.nanosec)) 
            # a variable that stores --> part index and time
            part_idx_time = [self.conveyor_part_count, [msg.header.stamp.sec,msg.header.stamp.nanosec]]
            self.conveyor_part_detected_list.append(part_idx_time)
            time.sleep(1)
            self.object_detected = False
            # self.predict_future_poses(self.conveyor_part_count, msg.header.stamp.sec, msg.header.stamp.nanosec)
            
    def predict_future_pose(self, index, sec, nanosec):
        ''' Computes the Pose of the Part 30s downstream 
        '''
        # speed = distance / time --->  0.2 m/s = (BreakBeam_y - PickZone_y) / 30 s
        # PickZone_y = 3.5 - 6 = - 2.5 m (always)
        BreakBeam_y = self.break_beam_y
        PickZone_y = BreakBeam_y - ( self.conveyor_speed * self.pickZone_time )         # pick up zone y-coordinate
        PickZone_sim_seconds = sec + self.pickZone_gap                                  # pick up zone arrival time
    
        # get x and z coordinates from Advanced Camera Logic camera Pose?
        future_y_time = [index, PickZone_y, [PickZone_sim_seconds, nanosec]]            # store index, future y, future time
        self.future_part_info.append(future_y_time)
        
        
        
            
def main(args=None):
    rclpy.init(args=args)
    breakbeam_subscriber = BreakBeamSubscriber()
    rclpy.spin(breakbeam_subscriber)
    breakbeam_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
