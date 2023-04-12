#!/usr/bin/env python3
__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'


"""
Task Manager Node - the Brain:
    -   Start and End Competition
    -   Track incoming orders and submit in a Sequence
    -   Track Parts and Trays
    -   Decide which Order to carry out
    -   Communicate with Floor Robot Node and Ceiling Robot Node
    -   Execute Order
    -   Track and deal with Agility Challenges
    
    
"""

import rclpy
import time
import numpy as np
import heapq as hq
from rclpy.node import Node
from rclpy.node import Subscription
from rclpy.qos import qos_profile_sensor_data
from typing import List

from ariac_msgs.msg import Order, BinParts, ConveyorParts
from ariac_msgs.msg import AdvancedLogicalCameraImage

from group5_rwa_3.store_and_submit import OrderClass
from group5_rwa_3.part_locations import ConveyerPartsClass, BinPartsClass
from group5_rwa_3.submit_order_nodes import SubmitOrderClient, SubmitStatePublisher
from group5_rwa_3.sensor_parts import alcPartClass


class TaskManager(Node):
    """
    Subscriber class for subscribing orders
    """
    # initialization, create all publisher and subscribers
    def __init__(self):
        super().__init__('task_manager_node')
     
        topic_incoming_orders = "/ariac/orders"                
        topic_initial_bin_parts = "/ariac/bin_parts"        
        topic_initial_conveyor_parts = "/ariac/conveyor_parts"        
        topic_alc_bins_left = "/ariac/sensors/alc_bin_left/image"
        topic_alc_bins_right = "/ariac/sensors/alc_bin_right/image"
         
        # Subscribe to Bin and Conveyer 
        # self.subscriber_initial_bin_parts: Subscription = self.create_subscription(
        #     BinParts,
        #     topic_initial_bin_parts,
        #     self.initial_bin_parts_callback,
        #     10
        # )
        
        # self.subscriber_initial_conveyor_parts: Subscription = self.create_subscription(
        #     ConveyorParts, 
        #     topic_initial_conveyor_parts,
        #     self.initial_conveyer_parts_callback, 
        #     10
        # )

        # subscribe to orders
        self.subscriber_incoming_orders: Subscription = self.create_subscription(
            Order,
            topic_incoming_orders,
            self.store_order_callback,
            10
        )
        
        # Subscriber alc - left bin image topic
        # self.subscriber_alc_left_bin: Subscription = self.create_subscription(
        #     AdvancedLogicalCameraImage,
        #     topic_alc_bins_left,
        #     self.left_bin_callback,
        #     qos_profile_sensor_data
        # )

        # Subscriber alc- right bin image topic
        # self.subscriber_alc_right_bin: Subscription = self.create_subscription(
        #     AdvancedLogicalCameraImage,
        #     topic_alc_bins_right,
        #     self.right_bin_callback,
        #     qos_profile_sensor_data
        # )
        
        # prevent unused variable warnings
        # self.subscriber_initial_bin_parts
        # self.subscriber_initial_conveyor_parts
        self.subscriber_incoming_orders
        # self.subscriber_alc_left_bin
        # self.subscriber_alc_right_bin
        
        #  -------------------------------| Track Insufficient Parts Challenge |-------------------------------
        # 5 rows, 4 columns --> matrix DS, stores 1 or 0
        self.calculated_total_parts: bool = False                              # calculate once, start of competition
        self.initial_bin_parts_received: bool = False                          # calculate once, start of competition
        self.initial_conveyer_parts_received: bool = False                     # calculate once, start of competition
                
        # ----------------------------------| Track Bin Parts |------------------------------------
        self.part_availablity_list: set = set()                                # stores every available part as part object - Part Type, Color, Pose 
        self.new_bin_list_left: set = set()                                    # tracks parts on left bin
        self.new_bin_list_right: set = set()                                   # tracks trays on right bin

        #  -------------------------------| Track Incoming Orders |-------------------------------
        self.priority_order_list: List = []
        self.order_list: List = []
        
        self.orders = []                            # standard list to store incoming orders
        hq.heapify(self.orders)                     # heapified list (heapq) to store incoming orders

        #  ---------------------------------| Submit Order |---------------------------------
        self.submit_state = False
        self.submit_state_publisher = SubmitStatePublisher(self.submit_state)   

        #  ---------------------------------| Robot State |---------------------------------
        self.floor_robot_available = True

        #  ---------------------------------| General |---------------------------------
        self.PartColor = dict([(0, "Red"), (1, "Green"),(2,"Blue"),(3,"Orange"),(4,"Purple")])
        self.PartType = dict([(10, "Battery"), (11, "Pump"),(12,"Sensor"),(13,"Regulator")])
        
    # ============================================================== #
    # |--------------------| Callback Methods |--------------------| #
    # ============================================================== #
    
    # callback function: store incoming orders in a list [heapq] of Order objects
    def store_order_callback(self, msg:Order) -> None:        
        # self.get_initial_remaining_total_parts()                                       # just initial check, others are updated at Order Execution
        self.get_logger().info("================================================================================================")
        self.get_logger().info("Incoming Order ==> ID: %s " % (msg.id))

        if msg.priority == 1: 
            self.priority_order_list.append([OrderClass(msg).id, OrderClass(msg).priority])
        else:
            self.order_list.append([OrderClass(msg).id, OrderClass(msg).priority])

        self.get_logger().info("Priority: %s " % self.priority_order_list)
        self.get_logger().info("General: %s " % self.order_list)  
          
        if (self.priority_order_list == None) and (self.order_list == None):
            return None

        # check if order can be submitted/executed
        if self.floor_robot_available == True:
            
            # determine next order to be submitted
            if self.priority_order_list:    
                self.popped_order = self.priority_order_list.pop()
            else:   
                self.popped_order = self.order_list.pop()
            
            self.get_logger().info("Popped Order: %s " % (self.popped_order))
            
            # determine if order can be exectued right now
            if self.check_execution_condition(self.popped_order) ==  False:
                self.get_logger().info(" Floor Robot is busy! ")
                self.priority_order_list.append(self.popped_order) if self.popped_order[1] == True else self.order_list.append(self.popped_order)
            else:
                self.get_logger().info(" Floor Robot is Executing Order! ")

       
    # callback function: identify and store parts in Bins [runs once]
    def initial_bin_parts_callback(self, msg:BinParts) -> None:
        if not self.initial_bin_parts_received:
            self.initial_bin_parts = BinPartsClass(msg)               # continuously updating
            self.initial_bin_parts_received = True

    # callback function: identify and store parts from Conveyor [runs once]
    def initial_conveyer_parts_callback(self, msg:ConveyorParts) -> None:
        if not self.initial_conveyer_parts_received:
            self.incoming_conveyer_parts = ConveyerPartsClass(msg)
            self.initial_conveyer_parts_received = True
    
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
            self.new_bin_list_right.add(self.part_info)
            # self.new_bin_list_left.add((self.PartColor[self.part_info.part_color], self.PartType[self.part_info.part_type]))

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
            self.new_bin_list_right.add(self.part_info)
            # self.new_bin_list_right.add((self.PartColor[self.part_info.part_color], self.PartType[self.part_info.part_type]))

        self.update_part_availablity_list()
    
    # ============================================================== #
    # |---------------| Insufficient Parts Methods |---------------| #
    # ============================================================== #
    
    # helper function: get initial total remaining competition parts                
    def get_initial_remaining_total_parts(self):
        # storing parts at the stand of the competition
        if self.calculated_total_parts == False:
            self.remaining_total_parts: np.array() = np.zeros((5,4))                         # to check if sufficient parts to FULLFILL order
            # loop through each bin and each part in the bin. add to DS
            for bin in self.initial_bin_parts.bins:
                for part in bin.parts:
                    self.remaining_total_parts[part.part.part_color, part.part.part_type - 10] += part.quantity
            # loop through each part on conveyor belt. add to DS
            for part in self.incoming_conveyer_parts.parts:
                self.remaining_total_parts[part.part.part_color, part.part.part_type - 10] += part.quantity
            self.calculated_total_parts = True
            self.print_remaining_total_parts()
            
  # helper function: update total remaining competition parts                
    def get_updated_remaining_total_parts(self):
        executed_order_parts = np.zeros((5,4)) 
        
        for part in self.popped_order[1].task.parts:            
            executed_order_parts[part.part_color, part.part_type - 10] += 1 
            
        for pColor,pType in np.argwhere(self.remaining_total_parts): 
            self.remaining_total_parts[pColor,pType] = int(self.remaining_total_parts[pColor,pType]) - int(executed_order_parts[pColor,pType])
        
        self.print_remaining_total_parts()

    # helper function: prints total remaining competition parts                
    def print_remaining_total_parts(self):
        # everything stored in the DS --> Initial Total Parts
        self.total_parts_list = np.argwhere(self.remaining_total_parts)
        self.get_logger().info("================================================================================")
        self.get_logger().info("Total Available Competition Parts: ")
        for pColor, pType in self.total_parts_list: 
            self.get_logger().info("    " + str(int(self.remaining_total_parts[pColor,pType])) + " x " + str(self.PartColor[pColor]) + " " + str(self.PartType[pType+10]))
        self.get_logger().info("================================================================================")
    
    # helper function: checks if sufficeint parts are present to complete kitting order
    def check_sufficient_order(self):
        
        # store Order's part requirement in Matrix DS
        order_parts = np.zeros((5,4)) 
        for part in self.popped_order[1].task.parts:            
            order_parts[part.part_color, part.part_type - 10] += 1 
        
        insufficiency_check_matrix = self.remaining_total_parts - order_parts     
        # outcomes --> 0: part exists || >0: part not needed. part || <0: insufficient parts
        if np.any(insufficiency_check_matrix < 0):  
            self.get_logger().warn("Insufficient parts to complete Kitting order")
            insufficient_parts = np.where(insufficiency_check_matrix < 0)
            insufficient_parts_shape = np.shape(insufficient_parts)[1]
            for i in range(insufficient_parts_shape):
                self.get_logger().info("    - missing color %s, type %s" % (self.PartColor[insufficient_parts[0][i]],  self.PartType[insufficient_parts[1][i]+10]))
            '''
            Add code to ensure that Reformed Order is executed
            Should complete the kitting order by ignoring the Missing Part
            Remove the missing Part [type, color] and Quadrant info from Order
            '''
        
    # ======================================================= #
    # |----------| Track Parts and Trays Methods |----------| #
    # ======================================================= #

    # helper function - track total available bin parts,  union(left bin parts, right bin parts)   
    def update_part_availablity_list(self):
        self.part_availablity_list = self.new_bin_list_left | self.new_bin_list_right
        self.get_logger().info(" Avaialble Bin Parts List --> %s" % self.part_availablity_list)
        
    # ============================================== #
    # |----------| Perform Task Methods |----------| #
    # ============================================== #
    
    # helper function - ensures that required Parts are on the bins before Order Execution
    def check_execution_condition(self, current_order):
        return False
         # store Order's part requirement in Matrix DS
        # order_parts = np.zeros((5,4)) 
        # for part in self.popped_order[1].task.parts:            
        #     order_parts[part.part_color, part.part_type - 10] += 1 
            
        # # store available bin parts in a 5 x 4 Data Structured
        # available_binPart = np.zeros((5,4))
        # for binPart in self.part_availablity_list:
        #     available_binPart[binPart.part.part_color. binPart.part.part_type - 10] += 1
                
        # execution_check_matrix = available_binPart - order_parts   
        # # outcomes --> 0: part exists || >0: part not needed. part || <0: insufficient parts
        # if np.any(execution_check_matrix < 0):  
        #     self.get_logger().warn("Insufficient parts to complete Kitting order")
        #     insufficient_parts = np.where(execution_check_matrix < 0)
        #     insufficient_parts_shape = np.shape(insufficient_parts)[1]
        #     for i in range(insufficient_parts_shape):
        #         self.get_logger().info("    - missing color %s, type %s" % (self.PartColor[insufficient_parts[0][i]],  self.PartType[insufficient_parts[1][i]+10]))
        #     '''
        #     Add the order back in heapq. And wait if the order is executable at a later date. 
        #     '''
        #     return False
        
        return True
    
    ''' Build upon from RWA2 '''
    def performTask(self, current_order):
        
        order_obj = current_order
        self.get_logger().info("Executing Order ==> ID: %s " % (order_obj.id))
        
        # Execute Kitting Order Task
        if order_obj.type==0: 
            self.get_logger().info("  --> Working on Kitting Order")

        self.submit_order(order_obj.id)  # call function to submit order, send in the ID of ORDER
        
        
    # ============================================== #
    # |----------| Submit Order Methods |----------| #
    # ============================================== #
    
    # calls the submit order client 
    def submit_order(self, order_id):
        submit_order_client = SubmitOrderClient()
        self.get_logger().info("Submitting Order ==> ID: %s " % (order_id))
        response = submit_order_client.send_request(order_id)
        submit_order_client.get_logger().info('Response from Submit Order server "%r", %s' % (response.success, response.message))
        self.get_logger().info("================================================================================================")
        self.get_updated_remaining_total_parts()
        # self.check_end_comp()
    
    # if heapq is empty, the state is published on /submit_state
    def check_end_comp(self):        
        if not self.orders:             # if the List is EMPTY
            self.submit_state = True
            self.submit_state_publisher.update_data(self.submit_state)
            rclpy.spin_once(self.submit_state_publisher)
        else:
            self.submit_state = False    

# main loop
def main(args=None):
    rclpy.init(args=args)
    task_manager: Node = TaskManager()
    rclpy.spin(task_manager)
    task_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
