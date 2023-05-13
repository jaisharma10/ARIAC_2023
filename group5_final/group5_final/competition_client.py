#!/usr/bin/env python3

__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'

"""
Holds Classes to Start and End client
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class StartCompClient(Node):
    
    '''
    the client instance responds to Trigger service when the topic:
    '/ariac/start_competition' is called. Request made through
    'send_empty_request' method is empty
    '''

    def __init__(self):
        super().__init__('start_comp_client')
        subscription_topic_name = '/ariac/start_competition'
        self.cli = self.create_client(Trigger, subscription_topic_name)
        while not self.cli.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()     

    def send_empty_request(self):
        self.get_logger().info('Starting Competition')
        self.get_logger().info(" ---------------------------------------------------- ")
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class EndCompClient(Node):
    
    '''
    the client instance responds to Trigger service when the 
    topic '/ariac/end_competition' is called. Request made through
    'send_empty_request' method is empty
    '''

    def __init__(self):
        super().__init__('end_comp_client')
        subscription_topic_name = '/ariac/end_competition'
        self.cli = self.create_client(Trigger, subscription_topic_name)
        while not self.cli.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

    def send_empty_request(self):
        self.get_logger().info(" ---------------------------------------------------- ")
        self.get_logger().info(" ---------------------------------------------------- ")
        self.get_logger().info(" ----                                            ---- ")
        self.get_logger().info(" ----                                            ---- ")
        self.get_logger().info(" ----                Ending Competition          ---- ")
        self.get_logger().info(" ----                                            ---- ")
        self.get_logger().info(" ----                                            ---- ")
        self.get_logger().info(" ---------------------------------------------------- ")
        self.get_logger().info(" ---------------------------------------------------- ")
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
