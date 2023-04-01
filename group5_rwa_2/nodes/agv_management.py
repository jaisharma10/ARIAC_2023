#!/usr/bin/env python3

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

    """
    # initialization, create all publisher and subscribers
    def __init__(self, number: int):
        super().__init__('agv_mover')
        self.AGV_number = number   
        print(f'/ariac/move_agv{self.AGV_number}')
        # self.service_client = f'/ariac/move_agv{self.AGV_number}'
        
        # self.topic_to_subscribe = f'/ariac/agv{self.AGV_number}_status'

        # Subscribe to Bin and Conveyer 
        self.mover_client = self.create_client(MoveAGV, f'/ariac/move_agv{self.AGV_number}')
        while not self.mover_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveAGV.Request()

    def move_agv_to(self, destination: str) -> None: 
        self.req.location = destination
        self.future = self.mover_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

  
def main():
    rclpy.init()

    minimal_client = ManageAGV(3)
    minimal_client.move_agv_to(3)
    minimal_client.get_logger().info( 'Moving AGV3 to location 3' )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



"""
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""