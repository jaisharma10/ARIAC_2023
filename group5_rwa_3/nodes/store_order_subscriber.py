#!/usr/bin/env python3

import rclpy
import time
import numpy as np
import heapq as hq
from rclpy.node import Node

from ariac_msgs.msg import Order, BinParts, ConveyorParts

from group5_rwa_3.store_and_submit import OrderClass
from group5_rwa_3.part_locations import ConveyerPartsClass, BinPartsClass
from group5_rwa_3.submit_order_nodes import SubmitOrderClient, SubmitStatePublisher



class OrderSubscriber(Node):
    """
    Subscriber class for subscribing orders
    """
    # initialization, create all publisher and subscribers
    def __init__(self):
        super().__init__('order_subscriber')
     
        self.orders = []                            # standard list to store incoming orders
        hq.heapify(self.orders)                     # heapified list (heapq) to store incoming orders
        
        # 5 rows, 4 columns --> matrix DS, stores 1 or 0
        self.total_parts = np.zeros((5,4))           # to check if sufficient parts to FULLFILL order
        self.stored_parts = np.zeros((5,4))          # to check if sufficient parts to START order
 
        self.calculated_total_parts = False
        self.bin_parts_received = False
        self.conveyer_parts_received = False

        self.PartColor = dict([(0, "Red"), (1, "Green"),(2,"Blue"),(3,"Orange"),(4,"Purple")])
        self.PartType = dict([(10, "Battery"), (11, "Pump"),(12,"Sensor"),(13,"Regulator")])
        self.PartType2 = dict([(0, "Battery"), (1, "Pump"),(2,"Sensor"),(3,"Regulator")])

        self.submit_state = False
        self.submit_state_publisher = SubmitStatePublisher(self.submit_state)   
        
        # Subscribe to Bin and Conveyer 
        self.sub_bin_parts = self.create_subscription(
            BinParts,
            '/ariac/bin_parts',
            self.bin_parts_callback,
            10
        )
        
        self.sub_conveyer_parts = self.create_subscription(
            ConveyorParts, 
            '/ariac/conveyor_parts', 
            self.conveyer_parts_callback, 
            10
        )

        # subscribe to orders
        self.sub_order = self.create_subscription(
            Order,
            "/ariac/orders",
            self.store_order_callback,
            10
        )
        
        self.sub_bin_parts
        self.sub_conveyer_parts
        self.sub_order

    # callback function: store incoming orders in a list [heapq] of Order objects
    def store_order_callback(self, msg:Order) -> None:        
        self.calc_total_parts() # must be updated constantly
        self.get_logger().info("================================================================================================")
        self.get_logger().info("Incoming Order ==> ID: %s " % (msg.id))
        time.sleep(15)
        hq.heappush(self.orders, (0 if msg.priority else 1, OrderClass(msg)))       
        if self.orders:                                                             # if heapq list is not empty
            self.popped_order = hq.heappop(self.orders)                                # pop the highest priority order
            if self.popped_order[1].type == 0:                        # if Kitting Order: check Insufficient Parts Challenge, then perform 
                self.check_sufficient_order()                                   
                self.performTask(self.popped_order[1],)          
            else:                                                  # else: perform Assembly or Combined order directly
                self.performTask(self.popped_order[1])                              
            
    # callback function: identify and store parts in Bins
    def bin_parts_callback(self, msg:BinParts) -> None:
        if not self.bin_parts_received:
            self.bin_parts = BinPartsClass(msg)               # continuously updating
            self.bin_parts_received = True

    # callback function: identify and store parts from Conveyor 
    def conveyer_parts_callback(self, msg:ConveyorParts) -> None:
        if not self.conveyer_parts_received:
            self.conveyer_parts = ConveyerPartsClass(msg)
            self.conveyer_parts_received = True
    
    # ============================================================== #
    # |---------------| Insufficient Parts Methods |---------------| #
    # ============================================================== #
    
    # helper function: stores TOTAL parts and CURRENT parts
    def calc_total_parts(self):
        # if self.calculated_total_parts == False:
        
        # loop through each bin and each part in the bin. add to DS
        for bin in self.bin_parts.bins:
            for part in bin.parts:
                self.total_parts[part.part.part_color, part.part.part_type - 10] += part.quantity
                self.stored_parts[part.part.part_color, part.part.part_type - 10] += part.quantity
                                    
        # loop through each part on conveyor belt. add to DS
        for part in self.conveyer_parts.parts:
            self.total_parts[part.part.part_color, part.part.part_type - 10] += part.quantity
        # self.calculated_total_parts = True
        
        total_parts_list = np.argwhere(self.total_parts)
        current_parts_list = np.argwhere(self.stored_parts)
        
        # self.get_logger().info out everything stored in the DS --> Initial Total Parts
        self.get_logger().info("================================================================================")
        self.get_logger().info("Total Available Parts: ")
        for j, k in total_parts_list: 
            self.get_logger().info("    " + str(int(self.total_parts[j,k])) + " x " + str(self.PartColor[j]) + " " + str(self.PartType2[k]))

        self.get_logger().info("================================================================================")
        self.get_logger().info("Current Available Parts:")
        for j, k in current_parts_list:  
            self.get_logger().info("    " + str(int(self.total_parts[j,k])) + " x " + str(self.PartColor[j]) + " " + str(self.PartType2[k]))
            
        self.get_logger().info("================================================================================")

    # helper function: checks if sufficeint parts are present to complete kitting order
    def check_sufficient_order(self):
        
        # store Order's part requirement in Matrix DS
        order_parts = np.zeros((5,4)) 
        for part in self.popped_order[1].task.parts:            
            order_parts[part.part_color, part.part_type - 10] += 1 
        
        remaining_parts = self.total_parts - order_parts     
        # outcomes --> 0: part exists || >0: part not needed. part || <0: insufficient parts
        if np.any(remaining_parts < 0):  
            self.get_logger().warn("Insufficient parts to complete Kitting order")
            insufficient_parts = np.where(remaining_parts < 0)
            # report missing parts
            missing_part_no = np.shape(insufficient_parts)[1]
            for i in range(missing_part_no):
                self.get_logger().info("    - missing color %s, type %s" % (self.PartColor[insufficient_parts[0][i]],  self.PartType[insufficient_parts[1][i]+10]))
        
    # ============================================== #
    # |----------| Perform Task Methods |----------| #
    # ============================================== #
    
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
        self.check_end_comp()
    
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
    order_subscriber = OrderSubscriber()
    rclpy.spin(order_subscriber)
    order_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
