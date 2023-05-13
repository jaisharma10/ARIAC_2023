#!/usr/bin/env python3
__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'


"""
Task Manager Node - the Brain:
    -   Start and End Competition
    -   Track incoming orders and submit in a Sequence
    -   Decide which Order to carry out
    -   Send Order to Floor Robot
    -   Execute Order
    
"""

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.node import Subscription, Publisher, Timer
from typing import List

from ariac_msgs.msg import Order, BinParts, ConveyorParts

from group5_final.part_locations import ConveyerPartsClass, BinPartsClass
from group5_final.submit_order_nodes import SubmitOrderClient, SubmitStatePublisher

from group5_msgs.msg import FloorRobotTask, CompletedOrder 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Trigger
from ariac_msgs.srv import MoveAGV as MoveAGVSrv



class TaskManager(Node):
    """
    Subscriber class for subscribing orders
    """
    # initialization, create all publisher and subscribers
    def __init__(self):
        super().__init__('task_manager_node')
     
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup() 
     
        topic_incoming_orders = "/ariac/orders"                
        topic_completed_order = "/competitor/completed_order"              
        topic_robot_task = "/competitor/robot_task"    
        topic_robot_task_priority = "/competitor/robot_task_priority"      

        timer_period = 2.0
                
        # subscribe to incoming orders []
        self.subscriber_incoming_orders: Subscription = self.create_subscription(
            Order,
            topic_incoming_orders,
            self.retrieve_orders_callback,
            10
        )
        
        # subscribe to completed Order [from Floor/Ceiling Robot Node]
        self.subscriber_complteted_order: Subscription = self.create_subscription(
            CompletedOrder,
            topic_completed_order,
            self.completed_order_callback,
            10
        )

        # publisher to send Order [ro Floor/Ceiling Robot Node]
        self.publisher_robot_task: Publisher = self.create_publisher(
            FloorRobotTask, 
            topic_robot_task,
            1
        )
        self.publisher_robot_task_priority: Publisher = self.create_publisher(
            FloorRobotTask, 
            topic_robot_task_priority,
            1
        )
        
        # timer callback: continuosly run Task Manager
        self.task_manager_timer: Timer = self.create_timer(
            timer_period, 
            callback = self.task_manager_callback)
        
        self.subscriber_incoming_orders
        self.subscriber_complteted_order
        self.publisher_robot_task
        self.publisher_robot_task_priority
        self.task_manager_timer
        
        # -------------------------------| Track End Competition |-------------------------------
        self.published_order_count: int = 0
        self.subscribed_order_count: int = 0
        
        # -------------------------------| Track Incoming Orders |-------------------------------
        self.orders: List = []
        self.announced_orders: List = []
        
        self.priority_order_list: List = []
        self.order_list: List = []
        
        # -------------------------------| Track Insufficient Parts Challenge |-------------------------------
        self.calculated_total_parts: bool = False                              # flag to calculate initial competition parts
        self.initial_bin_parts_received: bool = False                          # flag to run 'initial bin parts callback' only once
        self.initial_conveyer_parts_received: bool = False                     # flag to run 'initial conveyor parts callback' only once
                
        #  ---------------------------------| Submit Order |---------------------------------
        self.submitted_order = None
        self.submit_state = False
        self.submit_state_publisher = SubmitStatePublisher(self.submit_state)   

        #  ---------------------------------| General |---------------------------------
        self.PartColor = dict([(0, "Red"), (1, "Green"),(2,"Blue"),(3,"Orange"),(4,"Purple")])
        self.PartType = dict([(10, "Battery"), (11, "Pump"),(12,"Sensor"),(13,"Regulator")])
        
    # ============================================================== #
    # |--------------------| Callback Methods |--------------------| #
    # ============================================================== #
    
    # callback function: retrieve and store incoming orders
    def retrieve_orders_callback(self, msg: Order) -> None:

        self.get_logger().info("================================================================================================")
        self.get_logger().info("Received Order ==> ID: %s" % (msg.id))
        self.get_logger().info("Priority ==> : %s" % (True if msg.priority ==1 else False))
        
        # Priority Check, add to list accordingly
        if msg.priority == 1: self.priority_order_list.append(msg)
        else: self.order_list.append(msg)
        
        self.get_logger().info("----------------------------------------")        
        self.get_logger().info("Priority Orders List Size: %s " % str(len(self.priority_order_list)) + "General Orders List Size: %s " % str(len(self.order_list)))
        self.get_logger().info("----------------------------------------")        

    # callback function: store incoming orders in a list [heapq] of Order objects
    def task_manager_callback(self) -> None:        
        
        current_order_list = []
        if self.priority_order_list: 
            for order in self.priority_order_list:
                current_order_list.append(order)

        # self.orders is a list that holds Announced Order 'msg'        
        if self.order_list: 
            for order in self.order_list:
                current_order_list.append(order)
        
        if current_order_list:
            for order in current_order_list:
                
                self.published_order_count += 1

                self.get_logger().info("Performing Order ==> ID: %s " % (order.id))
                
                if order.type == Order.KITTING:
                    # check Part Sufficiency Challenge
                    self.get_logger().info("Order Type ==> Kitting Task ")
                    self.get_logger().info("----------------------------------------")
                    self.get_logger().info("----------------------------------------")

                    # "convert to FloorRobotTask message type"
                    msgOrder = FloorRobotTask()
                    msgOrder.id = order.id
                    msgOrder.type = FloorRobotTask.KITTING
                    msgOrder.kitting_task = order.kitting_task
                    msgOrder.priority = order.priority
                    if order.priority==False:
                             
                    # Publish the message (topic --> /competitor/floor_robot_task')
                        self.get_logger().info(" Publishing Order to Topic: %s " % ("/competitor/robot_task"))
                        self.publisher_robot_task.publish(msgOrder)  # SEND 'ORDER' TO FLOOR ROBOT NODE
                    if order.priority==True:
                             
                    # Publish the message (topic --> /competitor/floor_robot_task')
                        
                        self.publisher_robot_task_priority.publish(msgOrder)  # SEND 'ORDER' TO FLOOR ROBOT NODE
                        self.get_logger().info(" Publishing Order to Topic: %s " % ("/competitor/robot_task_priority"))
                
                if order.type == Order.ASSEMBLY:
                    self.get_logger().info("Order Type ==> Assembly Task ")
                    self.get_logger().info("----------------------------------------")
                
                if order.type == Order.COMBINED:
                    self.get_logger().info("Order Type ==> Combined Task ")
                    self.get_logger().info("----------------------------------------")
                    
                    # "convert to FloorRobotTask message type"
                    msgOrder = FloorRobotTask()
                    msgOrder.id = order.id
                    msgOrder.type = FloorRobotTask.COMBINED
                    msgOrder.combined_task = order.combined_task
                    msgOrder.priority = order.priority
                
                    if order.priority==False:
                             
                    # Publish the message (topic --> /competitor/floor_robot_task')
                        self.get_logger().info(" Publishing Order to Topic: %s " % ("/competitor/robot_task"))
                        self.publisher_robot_task.publish(msgOrder)  # SEND 'ORDER' TO FLOOR ROBOT NODE
                    if order.priority==True:
                             
                    # Publish the message (topic --> /competitor/floor_robot_task')
                        self.publisher_robot_task_priority.publish(msgOrder)  # SEND 'ORDER' TO FLOOR ROBOT NODE
                        self.get_logger().info(" Publishing Order to Topic: %s " % ("/competitor/robot_task_priority"))

                self.announced_orders.append(order)     # holds order in the correct message type.
                # remove from General/Priority Lists
                if order.priority == 1: self.priority_order_list.remove(order)
                else: self.order_list.remove(order)  
                
        if self.submitted_order is not None:
            for order in self.announced_orders:
                if order.id == self.submitted_order.order_id:
                    self.subscribed_order_count += 1
                    self.submit_order(order.id)
                    
            self.submitted_order = None

        return None  
                      
    # callback function: track orders that are performed by Floor Robot (completed orders)
    def completed_order_callback(self, msg) -> None:
        self.submitted_order = msg
    
    
    # ============================================== #
    # |----------| Submit Order Methods |----------| #
    # ============================================== #
    
    # calls the submit order client 
    def submit_order(self, order_id):
        submit_order_client = SubmitOrderClient()
        self.get_logger().info("================================================================================================")
        self.get_logger().info("Submitting Order ==> ID: %s " % (order_id))
        self.get_logger().info("================================================================================================")
        submit_order_client.send_request(order_id)
        self.check_end_comp()
    
    # if heapq is empty, the state is published on /submit_state
    def check_end_comp(self):         
        if (self.published_order_count == self.subscribed_order_count):
            self.submit_state = True
            self.submit_state_publisher.update_data(self.submit_state)
            rclpy.spin_once(self.submit_state_publisher)
  

# main loop
def main(args=None):
    rclpy.init(args=args)
    task_manager: Node = TaskManager()
    rclpy.spin(task_manager)
    task_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    