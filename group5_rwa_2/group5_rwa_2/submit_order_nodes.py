#!/usr/bin/env python3
__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ariac_msgs.srv import SubmitOrder  

class SubmitOrderClient(Node):
    """
    Client class - Submits Order ID on the ROS Topic '/ariac/submit_order'
    """
    
    # initialization
    def __init__(self):
        super().__init__('sub_order_client')
        self.cli = self.create_client(SubmitOrder, '/ariac/submit_order')
        while not self.cli.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info('service not available, waiting again...')
        self.req = SubmitOrder.Request()

    # sent request to ros service
    def send_request(self, id):
        self.req.order_id = id
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class SubmitStatePublisher(Node):
    """
    Publisher class - Publishes submission state (True/False) on ROS TOPIC /submit_state
    """
    
    # initialization
    def __init__(self, data):
        super().__init__('submit_state_publisher')
        self.data = data
        self.publisher_ = self.create_publisher(Bool, 'submit_state', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # update topic data
    def update_data(self, data_update):
        self.data = data_update

    # callback function for publishing submit_state topic
    def timer_callback(self):
        msg = Bool()
        msg.data = self.data
        self.publisher_.publish(msg)
        