#!/usr/bin/env python3
__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'

""" 
Runs the node that continuosly looking for incoming orders and storing it
in a Data Structure
"""

import rclpy
import heapq as hq
from rclpy.node import Node
from ariac_msgs.msg import Order
from group5_rwa_1.order_classes import OrderClass
from group5_rwa_1.submit_order_nodes import SubmitOrderClient, SubmitStatePublisher

class AnnouncedOrderSubscriber(Node):
    """
    Subscriber class for subscribing orders
    """
    # initialization, create all publisher and subscribers
    def __init__(self):
        super().__init__('announced_order_subscriber')
        self.orders = []
        hq.heapify(self.orders)
        self.submit_state = False
        self.submit_state_publisher = SubmitStatePublisher(self.submit_state)
        self.sub_order = self.create_subscription(
            Order,
            "/ariac/orders",
            self.store_order_callback,
            10
        )
        self.sub_order

    # callback function: store incoming order topics to list of Order objects
    def store_order_callback(self, msg:Order):
        self.get_logger().info("Incoming Order ==> ID: %s " % (msg.id))
        hq.heappush(self.orders, (0 if msg.priority else 1, OrderClass(msg)))       
        if self.orders:                                                             # if heapq list is not empty
            self.pop_order = hq.heappop(self.orders)                                # pop the highest priority order
            self.pop_order_id = self.pop_order[1].id                                # get the id of popped order
            self.submit_order()                                                     # call function to submit order

    # calls the submit order client
    def submit_order(self):
        submit_order_client = SubmitOrderClient()
        self.get_logger().info("Submitting Order ==> ID: %s " % (self.pop_order_id))
        response = submit_order_client.send_request(self.pop_order_id)
        submit_order_client.get_logger().info(
            'Response from Submit Order server "%r", %s' % (response.success, response.message))
        self.get_logger().info(" --------------------------------------------------- ")
        self.check_end_comp()
    
    # if heapq is empty, the state is published on /submit_state
    def check_end_comp(self):        
        if not self.orders:  # if empty List
            self.submit_state = True
            self.submit_state_publisher.update_data(self.submit_state)
            rclpy.spin_once(self.submit_state_publisher)
        else:
            self.submit_state = False            
     
# main loop
def main(args=None):
    rclpy.init(args=args)
    order_subscriber = AnnouncedOrderSubscriber()
    rclpy.spin(order_subscriber)
    order_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    