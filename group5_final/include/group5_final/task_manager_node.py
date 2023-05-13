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

from competitor_interfaces.msg import FloorRobotTask, CompletedOrder 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Trigger
from ariac_msgs.srv import MoveAGV as MoveAGVSrv



class TaskManager(Node):
    """
    Subscriber class for order mangement
    """
    # initialization, create all publisher and subscribers
    def __init__(self):
        super().__init__('task_manager_node')
     
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup() 
     
        topic_incoming_orders = "/ariac/orders"                
        topic_initial_bin_parts = "/ariac/bin_parts"        
        topic_initial_conveyor_parts = "/ariac/conveyor_parts"  
        topic_completed_order = "/competitor/completed_order"              
        topic_robot_task = "/competitor/robot_task"    
        topic_robot_task_priority = "/competitor/robot_task_priority"      

        timer_period = 2.0
                
        # Subscribe to Initial Bin and Conveyer Parts
        self.subscriber_initial_bin_parts: Subscription = self.create_subscription(
            BinParts,
            topic_initial_bin_parts,
            self.initial_bin_parts_callback,
            10
        )
        
        self.subscriber_initial_conveyor_parts: Subscription = self.create_subscription(
            ConveyorParts, 
            topic_initial_conveyor_parts,
            self.initial_conveyer_parts_callback, 
            10
        )
                
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
        
        self.subscriber_initial_bin_parts
        self.subscriber_initial_conveyor_parts
        self.subscriber_incoming_orders
        self.subscriber_complteted_order
        self.publisher_robot_task
        self.publisher_robot_task_priority
        self.task_manager_timer
        
        
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

        """
        callback function: retrieve and store incoming orders
        """
        if self.calculated_total_parts == False:
            self.get_initial_remaining_total_parts()
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
        """
        callback for publishing order to cpp file
        publishes a msg of task type to the topic
        "/competitor/robot_task"
        or
        "/competitor/robot_task_priority"
        """     
        
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
                
                self.get_logger().info("Performing Order ==> ID: %s " % (order.id))
                
                if order.type == Order.KITTING:
                    # check Part Sufficiency Challenge
                    self.get_logger().info("Order Type ==> Kitting Task ")
                    self.get_logger().info("----------------------------------------")
                    order = self.check_sufficient_order(order)
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
                    if order.type == Order.KITTING:
                        # add AGV services here
                        agv = order.kitting_task.agv_number
                    self.submit_order(order.id)
                    
            self.submitted_order = None

        return None  
                     
    # callback function: identify and store parts in Bins [runs once]
    def initial_bin_parts_callback(self, msg:BinParts) -> None:
        """
        callback for bin parts
        """
        if not self.initial_bin_parts_received:
            self.initial_bin_parts = BinPartsClass(msg)               # continuously updating
            self.initial_bin_parts_received = True

    # callback function: identify and store parts from Conveyor [runs once]
    def initial_conveyer_parts_callback(self, msg:ConveyorParts) -> None:
        """

        callback for conveyor parts
        """
        if not self.initial_conveyer_parts_received:
            self.incoming_conveyer_parts = ConveyerPartsClass(msg)
            self.initial_conveyer_parts_received = True   
      
    # callback function: track orders that are performed by Floor Robot (completed orders)
    def completed_order_callback(self, msg) -> None:
        """

        order submission
        """
        self.submitted_order = msg

       
    # ============================================================== #
    # |---------------| Insufficient Parts Methods |---------------| #
    # ============================================================== #
        
    # helper function: get initial total remaining competition parts                
    def get_initial_remaining_total_parts(self):

        '''
        get bin parts, conveyor parts --> store in self.remaining_total_parts
        continuously update the parts
        '''
        # storing parts at the stand of the competition
        self.remaining_total_parts: np.array() = np.zeros((5,4))                         # to check if sufficient parts to FULLFILL order
        
        # loop through each bin and each part in the bin. add to DS
        for bin in self.initial_bin_parts.bins:
            for part in bin.parts:
                self.remaining_total_parts[part.part.part_color, part.part.part_type - 10] += part.quantity
        
        # loop through each part on conveyor belt. add to DS
        for part in self.incoming_conveyer_parts.parts:
            self.remaining_total_parts[part.part.part_color, part.part.part_type - 10] += part.quantity
        
        self.calculated_total_parts = True
        # self.print_remaining_total_parts(self.remaining_total_parts)
                
    # helper function: updates total remaining competition parts                
    def update_remaining_total_parts(self, executed_order):
        '''
        remaining_total_parts = remaining_total_parts - executed_order_parts
        '''
        executed_order_parts = np.zeros((5,4)) 
        
        for part in executed_order.kitting_task.parts:    # NEEDS to be Changed for [Assembly, Combined]        
            executed_order_parts[part.part.color, part.part.type - 10] += 1 
            
        for pColor,pType in np.argwhere(self.remaining_total_parts): 
            self.remaining_total_parts[pColor,pType] = int(self.remaining_total_parts[pColor,pType]) - int(executed_order_parts[pColor,pType])
        
        # self.print_remaining_total_parts(self.remaining_total_parts)

    # helper function: prints total remaining competition parts                
    def print_remaining_total_parts(self, print_parts):

        """
        everything stored in the DS --> Initial Total Parts
        """
        
        self.total_parts_list = np.argwhere(print_parts)
        self.get_logger().info("----------------------------------------")
        self.get_logger().info("Remaining Available Competition Parts: ")
        for pColor, pType in self.total_parts_list: 
            self.get_logger().info("    " + str(int(print_parts[pColor,pType])) + " x " + str(self.PartColor[pColor]) + " " + str(self.PartType[pType+10]))
        self.get_logger().info("----------------------------------------")
    
    # helper function: checks if sufficeint parts are present to complete kitting order
    def check_sufficient_order(self, current_order):


        '''
        Use array to check the insufficient parts
        insufficiency_check_matrix = self.remaining_total_parts - order_part_req  
        if any value inside matrix is '< 0', parts are Insufficient
            --> update order
        '''

        order_part_req = np.zeros((5,4)) 
        
        # stores parts as [colour #, type #]
        order_req_list = []
        remove_parts_list = []  

        # store Order Parts --> List and Matrix
        for part in current_order.kitting_task.parts:            
            order_part_req[part.part.color, part.part.type - 10] += 1 
            order_req_list.append([part.part.color, part.part.type])
        
        # outcomes --> 0: part exists || >0: part not needed. part || <0: insufficient parts 
        insufficiency_check_matrix = self.remaining_total_parts - order_part_req     
        
        for part in current_order.kitting_task.parts:
            self.get_logger().info("Part Requirements for Current Order --> ", once=True)
            self.get_logger().info("        - quadrant %s: %s %s" % (part.quadrant, self.PartColor[part.part.color],  self.PartType[part.part.type]))

        # if total number of parts are insufficient
        if np.any(insufficiency_check_matrix < 0):  
            self.get_logger().warn("Challenge FAILED: Insufficient Parts to complete Kitting Order!")
            insufficient_parts = np.where(insufficiency_check_matrix < 0)
            
            # store Remove Parts --> List and Matrix
            for i in range(np.shape(insufficient_parts)[1]):
                self.get_logger().warn("        - %s %s is unavailable." % (self.PartColor[insufficient_parts[0][i]],  self.PartType[insufficient_parts[1][i]+10]))
                remove_parts_list.append([insufficient_parts[0][i], insufficient_parts[1][i]+10])
            
            # extract Required Parts List
            updated_list = [i for i in order_req_list if i not in remove_parts_list]        
            updated_order = current_order

            # Remove the Parts
            for req_part in updated_order.kitting_task.parts:
                if [req_part.part.color, req_part.part.type] not in updated_list:
                    updated_order.kitting_task.parts.remove(req_part)

            # Print Modified Order Rquirements
            for part in updated_order.kitting_task.parts:
                self.get_logger().info("Submit with missing parts. Modified Order: --> ", once=True)                               
                self.get_logger().info("        - quadrant %s: %s %s" % (part.quadrant, self.PartColor[part.part.color],  self.PartType[part.part.type]))

            '''
            Here add functionality to directly Submit order if No Parts are Available
            '''

            return updated_order
        
        else:
            # no need to modify order
            self.get_logger().info("Challenge PASSED: Sufficient Parts to complete Kitting Order")
            return current_order
    
    # ============================================================== #
    # |-----------------------| AGV Methods NOT WORKING |----------------------| #
    # ============================================================== #
    
    # helper function: lock agv 
    def lock_agv(self, agv_num):
        """
        Lock agv service
        """
        self.get_logger().info("Locking AGV # " + str(agv_num))
        service_name = '/ariac/agv' + str(agv_num) + '_lock_tray'
        lock_agv_client = self.create_client(Trigger, service_name, callback_group=self.client_cb_group)
        request = Trigger.Request()
        _ = lock_agv_client.call(request) 
        
    # helper function: move agv to destination
    def move_agv(self, agv_num, destination):   

        """
        move agv service
        """
        self.get_logger().info("Moving AGV #" + str(agv_num) + " to " + str(destination))
        service_name = '/ariac/move_agv' + str(agv_num)
        move_agv_client = self.create_client(MoveAGVSrv, service_name, callback_group=self.client_cb_group)
        request = MoveAGVSrv.Request()
        request.location = destination
        _ = move_agv_client.call(request) 
    
    # ============================================== #
    # |----------| Submit Order Methods |----------| #
    # ============================================== #
    
    # calls the submit order client 
    def submit_order(self, order_id):
        """
        submit order function for submitting the order
        """
        submit_order_client = SubmitOrderClient()
        self.get_logger().info("Submitting Order ==> ID: %s " % (order_id))
        response = submit_order_client.send_request(order_id)
        # submit_order_client.get_logger().info('Response from Submit Order server "%r", %s' % (response.success, response.message))
        self.get_logger().info("================================================================================================")
    
    # if heapq is empty, the state is published on /submit_state
    def check_end_comp(self): 
        """
        End cometetion function
        Verify all the order are executed and then
        end competetion
        """
        if (self.order_list == []) and (self.priority_order_list == []):             # if the List is EMPTY
            self.submit_state = True
            self.submit_state_publisher.update_data(self.submit_state)
            rclpy.spin_once(self.submit_state_publisher)
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
    