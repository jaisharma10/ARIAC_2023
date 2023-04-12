#!/usr/bin/env python3
__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'

"""
Node to Perform all Conveyor Part Processing Tasks:
    -   Use Break Beam to track incoming parts
    -   Use Advanced Logic Camera to identify incoming parts
    -   Combine Time,  TimeStamp from Break Beam and Part Pose from
"""

#!/usr/bin/env python3
        

"""
Node to Subscribe to Break Beam Sensor
"""

""" 
To Do:
    - reduce ALC_conveyor to ensure only one part is read in its FOV
    - read camera pose 
    - perform tf tranformation for parts and trays
        - camera frame to world frame
    - store 'world frame' pose instead of camera frame pose
    - integrate sensor/camera related fucntionalities with main Task Manager node
    - fix class loops
"""

import time
import rclpy
from typing import List
from rclpy.node import Node
from rclpy.node import Subscription
from enum import Enum
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data

from builtin_interfaces.msg import Time
from ariac_msgs.msg import BreakBeamStatus 
from ariac_msgs.msg import AdvancedLogicalCameraImage
from group5_rwa_3.sensor_parts import alcPartClass
from group5_rwa_3.sensor_parts import alcTrayClass

class LogLevel(Enum):
    """
    An Enum class for identifying log severity level for logging the msg data using ROS loggers
    """
    DEBUG = 1,
    INFO = 2,
    WARNING = 3,
    ERROR = 4,
    FATAL = 5

class SensorSubscriber(Node):
    
    def __init__(self):
        super().__init__('sensor')
        
        subscription_topic_bb_conveyor = "/ariac/sensors/breakbeam_conveyor/status"
        subscription_topic_alc_conveyor = "/ariac/sensors/alc_conveyor/image"
        subscription_topic_alc_kitting_tray_left = "/ariac/sensors/alc_kitting_tray_left/image"
        subscription_topic_alc_kitting_tray_right = "/ariac/sensors/alc_kitting_tray_right/image"
        subscription_topic_alc_bins_left = "/ariac/sensors/alc_bin_left/image"
        subscription_topic_alc_bins_right = "/ariac/sensors/alc_bin_right/image"

        # # Subscriber break beam - status topic
        self.break_beam_conveyor: Subscription = self.create_subscription(
            BreakBeamStatus,
            subscription_topic_bb_conveyor,
            self.breakbeam_conveyor_callback,
            qos_profile_sensor_data
        )
        
        # # Subscriber alc - conveyor image topic
        self.alc_conveyor: Subscription = self.create_subscription(
            AdvancedLogicalCameraImage,
            subscription_topic_alc_conveyor,
            self.alc_conveyor_callback,
            qos_profile_sensor_data
        )
        
        # Subscriber alc - left kitting tray image topic
        self.alc_left_kitting_tray: Subscription = self.create_subscription(
            AdvancedLogicalCameraImage,
            subscription_topic_alc_kitting_tray_left,
            self.left_kitting_tray_callback,
            qos_profile_sensor_data
        )

        # Subscriber alc- right kitting tray image topic
        self.alc_right_kitting_tray: Subscription = self.create_subscription(
            AdvancedLogicalCameraImage,
            subscription_topic_alc_kitting_tray_right,
            self.right_kitting_tray_callback,
            qos_profile_sensor_data
        )

        # Subscriber alc - left bin image topic
        self.alc_left_bin: Subscription = self.create_subscription(
            AdvancedLogicalCameraImage,
            subscription_topic_alc_bins_left,
            self.left_bin_callback,
            qos_profile_sensor_data
        )

        # # # Subscriber alc- right bin image topic
        self.alc_right_bin: Subscription = self.create_subscription(
            AdvancedLogicalCameraImage,
            subscription_topic_alc_bins_right,
            self.right_bin_callback,
            qos_profile_sensor_data
        )
        
        # prevent unused variable warnings
        self.break_beam_conveyor                      
        self.alc_conveyor                             
        self.alc_left_kitting_tray
        self.alc_right_kitting_tray
        self.alc_left_bin
        self.alc_right_bin

        #  ---------------------------------| General |---------------------------------

        self.PartColor = dict([(0, "Red"), (1, "Green"),(2,"Blue"),(3,"Orange"),(4,"Purple")])
        self.PartType  = dict([(10, "Battery"), (11, "Pump"),(12,"Sensor"),(13,"Regulator")])
        
        #  ---------------------------------| Conveyors |---------------------------------
        
        self.bb_conveyor_part_count: int = 0              # increment for each part breaking the beam
        self.alc_conveyor_part_count: int = 0             # increment for each part detected by the alc 
        self.object_detected: bool = False                # track if beam is broken
                
        self.conveyor_speed: float = 0.2                    # speed of conveyor (m/s)
        self.pick_up_time_gap: float = 30.0                 # pick zone is selected to be T seconds downstream
        self.break_beam_y: float = 3.5                      # y position of break beam sensor
        self.pick_up_y_value: float = self.get_pickup_y()   # gives pickup y coordinate (constant for all parts)

        self.bb_time_stamp_list: List = []                  # store occurence of detected part and time
        self.alc_pose_type_list: List = []                  # store future pose of detected part and time
        
        self.pick_up_dict: dict = {}
        
        # ----------------------------------| Trays |------------------------------------
        
        self.tray_availablity_list: set = set()                                # stores every available tray as tray object - ID, LEFT/RIGHT, 
        self.new_tray_list_left: set = set()                                   # tracks trays on right kitting table
        self.new_tray_list_right: set = set()                                  # tracks trays on right kitting table

        # ----------------------------------| Bins |------------------------------------
        
        self.part_availablity_list: set = set()                                # stores every available part as part object - Part Type, Color, Pose 
        self.new_bin_list_left: set = set()                                    # tracks parts on left bin
        self.new_bin_list_right: set = set()                                   # tracks trays on right bin


    # helper function - initializes y coordinate of part at Pick Up (constant for all parts)
    def get_pickup_y(self):
        return self.break_beam_y - ( self.conveyor_speed * self.pick_up_time_gap)         # pick up zone y-coordinate
    
    # ============================================================== #
    # |--------------------| Callback Methods |--------------------| #
    # ============================================================== #
     
    # callback - break beam sensor
    def breakbeam_conveyor_callback(self, msg: BreakBeamStatus):
        '''
            Callback for the breakbeam_conveyor topic 
            Tracks the number of parts that cross the break beam
        '''
        
        if msg.object_detected: self.object_detected =  True
        
        if self.object_detected: 
            self.bb_conveyor_part_count += 1
            self.get_logger().info("================================================ ")             
            self.get_logger().info(" Break Beam: Part (%s) - Time Stamp: %s sec and %s nanosec"  %  (self.bb_conveyor_part_count, msg.header.stamp.sec,msg.header.stamp.nanosec)) 
            self.object_detected = False
            time.sleep(1.5) 
            self.store_future_time(self.bb_conveyor_part_count, msg.header.stamp)
          
    # callback - advanced logic camera - conveyor
    def alc_conveyor_callback(self, msg: AdvancedLogicalCameraImage):
        '''
            Callback for the alc_conveyor topic 
            Tracks the parts in its fov and stores poses
            Stores as an object of AdvancedLogicalCamera_Part_Class
        '''
                
        if msg.part_poses == []:
            # self.get_logger().info('No Part in Camera FOV')
            return None

        for i, partPose in enumerate(msg.part_poses):
            self.alc_conveyor_part_count += 1
            self.part_info = alcPartClass(partPose)
            self.get_logger().info(" Advanced Camera: Part (%s) - a %s %s." %  (self.alc_conveyor_part_count, self.PartColor[self.part_info.part_color],
                                                                                        self.PartType[self.part_info.part_type]))
            self.store_future_pose(self.alc_conveyor_part_count, self.part_info)
            time.sleep(4)

    # callback - advanced logic camera - left kitting tray
    def left_kitting_tray_callback(self, msg: AdvancedLogicalCameraImage):
        '''
            Callback for LEFT kitting table ALC
            Store as tray object
        '''
            
        self.new_tray_list_left = set()

        if msg.tray_poses == []:
            # self.get_logger().info('No Tray in Camera FOV')
            return None
        
        for left_tray_id_pose in (msg.tray_poses):
            self.left_tray_info = alcTrayClass(left_tray_id_pose)
            self.left_tray_info.side = 'Left'
            # self.get_logger().info(" Tray Detected on Left Kitting Tray - ID: %s, Side: %s." % (self.left_tray_info.tray_id, self.left_tray_info.side))
            self.new_tray_list_left.add(self.left_tray_info.tray_id)
            
        self.update_tray_availablity_list()

    # callback - advanced logic camera - right kitting tray
    def right_kitting_tray_callback(self, msg: AdvancedLogicalCameraImage):
        '''
            Callback for RIGHT kitting table ALC
            Store as tray object
        '''

        self.new_tray_list_right = set()        # only save new incoming data.
                
        if msg.tray_poses == []:
            # self.get_logger().info('No Tray in Camera FOV')
            return None

        for right_tray_id_pose in (msg.tray_poses):
            self.right_tray_info = alcTrayClass(right_tray_id_pose)
            self.right_tray_info.side = 'Right'
            # self.get_logger().info(" Tray Detected on Right Kitting Tray - ID: %s, Side: %s." % (self.right_tray_info.tray_id, self.right_tray_info.side))
            self.new_tray_list_right.add(self.right_tray_info.tray_id)
            
        self.update_tray_availablity_list()
       
    # callback - advanced logic camera - left bin 
    def left_bin_callback(self, msg: AdvancedLogicalCameraImage):
        '''
            Callback for LEFT bin using ALC
            Store 'part objects' by scanning entire Left Bin
        '''

        self.new_bin_list_left = set()

        if msg.part_poses == []:
            # self.get_logger().info('No Part in Camera FOV')
            return None

        for i, partPose in enumerate(msg.part_poses):
            self.part_info = alcPartClass(partPose)
            # self.bin_list_right.add(self.part_info)
            self.new_bin_list_left.add((self.PartColor[self.part_info.part_color], self.PartType[self.part_info.part_type]))

        self.update_part_availablity_list()  
            
    # callback - advanced logic camera - right bin 
    def right_bin_callback(self, msg: AdvancedLogicalCameraImage):
        '''
            Callback for RIGHT bin using ALC
            Store 'part objects' by scanning entire Right Bin
        '''

        self.new_bin_list_right = set()

        if msg.part_poses == []:
            self.get_logger().info('No Part in Camera FOV')
            return None

        for i, partPose in enumerate(msg.part_poses):
            self.part_info = alcPartClass(partPose)
            # self.get_logger().info(" Right Bin ALC: Part - a %s %s." %  (self.PartColor[self.part_info.part_color],
                                                                            # self.PartType[self.part_info.part_type]))
            # self.bin_list_right.add(self.part_info)
            self.new_bin_list_right.add((self.PartColor[self.part_info.part_color], self.PartType[self.part_info.part_type]))

        self.update_part_availablity_list()

            
    # ============================================================== #
    # |----------------| Conveyor Helper Methods |-----------------| #
    # ============================================================== #

    # helper function - store future/pickup timestamp    
    def store_future_time(self, index, bb_time):
        bb_time.sec += int(self.pick_up_time_gap)                       # increment Break Beam time
        self.pick_up_dict[index] = [bb_time]                            # saves as builtin_interfaces.msg.Time object
        self.get_logger().info("pick up dictionary bb Time %s" % self.pick_up_dict)

    # helper function - store future/pickup pose and type        
    def store_future_pose(self, index, part_info):
        part_info.pose.position.y = self.pick_up_y_value              # update downstream/predicted y-coordinate for each part
        self.pick_up_dict[index].append(part_info)                    # saves as custom alcPartClass object
        self.get_logger().info("pick up dictionary alc Pose %s" % self.pick_up_dict)
        
    # ============================================================== #
    # |-------------------| Tray Helper Methods |------------------| #
    # ============================================================== #       
    
    # helper function - track total available trays,  union(left table trays, right table trays)   
    def update_tray_availablity_list(self):
        self.get_logger().info(" Avaialble Trays List --> %s" % (self.new_tray_list_left | self.new_tray_list_right))
    
    # ============================================================== #
    # |-------------------| Bins Helper Methods |------------------| #
    # ============================================================== #       
    
    # helper function - track total available bin parts,  union(left bin parts, right bin parts)   
    def update_part_availablity_list(self):
        self.get_logger().info(" Avaialble Bin Parts List --> %s" % (self.new_bin_list_left | self.new_bin_list_right))
          
    def log(self, msg: str, level: LogLevel = LogLevel.INFO) -> None:
            """
            Implementation by the derived class entity for logging using ROS logger

            :param level: LogLevel
                severity level of the msg
            :param msg: str
                data that need to be handled or logged
            :return: None
            """
                                
            if level == LogLevel.DEBUG:
                self.get_logger().debug(msg)
            elif level == LogLevel.WARNING:
                self.get_logger().warn(msg)
            elif level == LogLevel.ERROR:
                self.get_logger().error(msg)
            elif level == LogLevel.INFO:
                self.get_logger().info(msg)
            elif level == LogLevel.FATAL:
                self.get_logger().fatal(msg)

            return None

  
def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()
    rclpy.spin(sensor_subscriber)
    sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
