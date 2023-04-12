#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ariac_msgs.msg import AdvancedLogicalCameraImage
from group5_rwa_3.sensor_parts import alcPartClass


"""

Node to read Advanced Logical Cameras 

"""


class AdvLogicalCameraSubscriber(Node):
    
    def __init__(self):
        super().__init__('advanced_logical_camera')
        
        subscription_topic_alc_conveyor = "/ariac/sensors/alc_conveyor/image"

        # Subscriber to the advanced logical camera - conveyor image topic
        self.alc_conveyor = self.create_subscription(
            AdvancedLogicalCameraImage,
            subscription_topic_alc_conveyor,
            self.alc_conveyor_callback,
            qos_profile_sensor_data
        )
        
        self.alc_conveyor                # prevent unused variable warnings
        self.conveyor_parts_list: dict = {}
        
        self.PartColor = dict([(0, "Red"), (1, "Green"),(2,"Blue"),(3,"Orange"),(4,"Purple")])
        self.PartType  = dict([(10, "Battery"), (11, "Pump"),(12,"Sensor"),(13,"Regulator")])
        
    def alc_conveyor_callback(self, msg: AdvancedLogicalCameraImage):
        '''
        Callback for the alc_conveyor topic 
        Tracks the parts in its fov and stores poses
        Stores as an object of AdvancedLogicalCamera_Part_Class
        '''
                
        self.get_logger().info(' =================================================================== ')                   

        if msg.part_poses == []:
            self.get_logger().info('No Part in Camera FOV')
            time.sleep(1)
            return None

        for i, partPose in enumerate(msg.part_poses):
            self.part_info = alcPartClass(partPose)
            self.get_logger().info('Part Detected!')
            self.get_logger().info('Index Entry -- %s' % i)
            self.get_logger().info('A %s %s has been Detected on the Conveyor!' %  (self.PartColor[self.part_info.part_color],
                                                                                        self.PartType[self.part_info.part_type]))
            self.get_logger().info('    part position -- %s' % self.part_info.pose.position)      
            self.get_logger().info('    part orientation -- %s' % self.part_info.pose.orientation)
            
            self.conveyor_parts_list[i] = self.part_info
            time.sleep(1)

        # self.get_logger().info('    conveyor parts list-- %s' % self.conveyor_parts_list)
        # self.get_logger().info(' =================================================================== ')                   
                 
     
def main(args=None):
    # print("Hey")
    rclpy.init(args=args)
    alc_subscriber = AdvLogicalCameraSubscriber()
    rclpy.spin(alc_subscriber)
    alc_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
