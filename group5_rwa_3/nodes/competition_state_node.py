#!/usr/bin/env python3

'''
Subscribes to /ariac/competition_state and calls the relevant Client
based on msg.competition_state type
Running in the background [2Hz]
'''

import time
import rclpy
from rclpy.node import Node
from rclpy.node import Subscription
from std_msgs.msg import Bool
from ariac_msgs.msg import CompetitionState
from group5_rwa_3.competition_client import StartCompClient, EndCompClient


class CompStateSubscriber(Node):
    
    """
    Competition State Subscriber: This subscribes to two topics, competition_state and the state of the order list. 
    This takes Node as a parent class. 
    
    """

    # initialization, create all subscribers
    def __init__(self):
        super().__init__('comp_state_subscriber')
        
        topic_competition_state = "/ariac/competition_state"
        topic_empty_list_tracker = "/submit_state"
        
        self.subscriber_state: Subscription = self.create_subscription(
            CompetitionState,
            topic_competition_state,
            self.competition_state_callback,
            2
        )

        self.subscriber_list: Subscription = self.create_subscription(
            Bool,
            topic_empty_list_tracker,
            self.empty_list_callback,
            2
        )
        
        self.subscriber_state  # prevent unused variable warnings
        self.subscriber_list   # prevent unused variable warnings
        
        self.submit_state: bool = False

    # callback function: performs task based on Competition State
    def competition_state_callback(self, msg:CompetitionState):
        
        """
        This call back function is for starting and ending the competition. 
        It is subscribing to a custom topic (created by us) having simple datatype of boolean. 
        It is supposed flag whether the order list is empty or not. 
        """
 
        # check if competition can START
        if msg.competition_state == CompetitionState.READY:            
            start_comp_client = StartCompClient()
            start_comp_client.send_empty_request()
            start_comp_client.destroy_node()

        # check if competition can END
        if (msg.competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE and 
            self.submit_state):
            time.sleep(1)
            end_comp_client = EndCompClient()
            end_comp_client.send_empty_request()
            end_comp_client.destroy_node()
            
    def empty_list_callback(self, msg:Bool):
        self.submit_state = msg.data 
    
def main(args=None):
    rclpy.init(args=args)
    time.sleep(1)
    comp_state_subscriber = CompStateSubscriber()
    rclpy.spin(comp_state_subscriber)
    comp_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
    
    
    
    
