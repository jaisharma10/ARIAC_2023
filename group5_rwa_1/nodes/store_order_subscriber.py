#!/usr/bin/env python3
__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'


import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from ariac_msgs.msg import Order
from ariac_msgs.srv import SubmitOrder  
from ariac_msgs.msg import CompetitionState
from group5_rwa_1.store_and_submit import OrderClass

class SubmitOrderClient(Node):
    """
    Clinet class for submitting order to submit_order service
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
    Publisher class for publishing submission state topic
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

class OrderSubscriber(Node):
    """
    Subscriber class for subscribing orders
    """
    # initialization, create all publisher and subscribers
    def __init__(self):
        super().__init__('order_subscriber')
        self.submitted = False
        self.orders = []
        self.submit_state = False
        self.submit_state_publisher = SubmitStatePublisher(self.submit_state)
        self.sub_order = self.create_subscription(
            Order,
            "/ariac/orders",
            self.store_order_callback,
            10
        )
        self.sub_comp_state = self.create_subscription(
            CompetitionState,
             "/ariac/competition_state",
            self.competition_state_callback,
            2
        )

    # callback function for storing incoming order topics to list of Order objects
    def store_order_callback(self, msg:Order):
        self.get_logger().info("Incoming Order ==> ID: %s || Type: %s" % (msg.id, msg.type))
        self.orders.append(OrderClass(msg))
    
    # callback function for checking order announcement, sort orders, and submit orders
    def competition_state_callback(self, msg:CompetitionState):
        if msg.competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE and self.submitted == False:
            self.get_logger().info(" ---------------------------------------------------- ")
            self.sort_order()
            self.submit_order()
            self.submitted = True
        self.submit_state_publisher.update_data(self.submit_state)
        rclpy.spin_once(self.submit_state_publisher)
            
    # function for sorting orders accoring to priority level
    def sort_order(self):
        # sort order according to priority
        priorities = []
        ids = []
        for i in range(len(self.orders)):
            priorities.append(self.orders[i].priority)
            ids.append(self.orders[i].id)
        sort = np.argsort(np.array(priorities).astype(int))
        self.ids_sorted = np.array(ids)[sort]
        self.ids_sorted = list(self.ids_sorted)

    # function which call SubmitOrderClient class to submit orders
    def submit_order(self):
        submit_order_client = SubmitOrderClient()
        while self.ids_sorted:
            temp_id = self.ids_sorted.pop()
            response = submit_order_client.send_request(temp_id)
            self.get_logger().info("Order ID: %s" % temp_id)
            submit_order_client.get_logger().info(
            'Response from server %r, %s' %
            (response.success, response.message))
        self.submit_state = True

# main loop
def main(args=None):
    rclpy.init(args=args)
    order_subscriber = OrderSubscriber()
    rclpy.spin(order_subscriber)
    order_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    