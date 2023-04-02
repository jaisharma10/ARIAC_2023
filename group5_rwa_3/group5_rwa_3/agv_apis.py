#!/usr/bin/env python3

__author__ = 'Pratik'
__maintainer__ = 'pratik94@terpmail.umd.edu'

import rclpy
from rclpy.node import Node

from ariac_msgs.msg import AGVStatus
from ariac_msgs.srv import MoveAGV


class ManageAGV(Node):
    """
    This class will provide methods to manage AGVs. 
    It will subscribe to relevant topics and services to manage and get status of AGVs. 

    Topic: /ariac/move_agv{n}
           /ariac/agv{n}_lock_tray
           /ariac/agv{n}_unlock_tray
           etc....

    Service: /ariac/agv{n}_status
             /agv{n}_controller/commands
             check

    """
    # initialization, create all publisher and subscribers
    def __init__(self, number: int):
        super().__init__('agv_mover')
        self.AGV_number = number   
        print(f'/ariac/move_agv{self.AGV_number}')

        # Subscribe to Bin and Conveyer 
        self.mover_client = self.create_client(MoveAGV, f'/ariac/move_agv{self.AGV_number}')
        while not self.mover_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveAGV.Request()

    def move_agv_to(self, destination: str) -> None: 
        self.req.location = destination
        self.future = self.mover_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
  


