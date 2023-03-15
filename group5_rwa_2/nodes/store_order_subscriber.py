#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from ariac_msgs.srv import SubmitOrder  

from ariac_msgs.msg import CompetitionState, Order, BinParts, ConveyorParts

from group5_rwa_2.store_and_submit import OrderClass
from group5_rwa_2.part_locations import ConveyerPartsClass, BinPartsClass
from group5_rwa_2.perform_task import floorRobotClass


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
        # self.ordersExecuted = False
        self.orders = []
        self.order_parts_array = []
        self.calculated_total_parts = False
        self.bin_parts_received = False
        self.conveyer_parts_received = False
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
        # subscribe to competition state
        self.sub_comp_state = self.create_subscription(
            CompetitionState,
             "/ariac/competition_state",
            self.competition_state_callback,
            2
        )


    # callback function: identify and store parts in Bins
    def bin_parts_callback(self, msg:BinParts):
        if self.bin_parts_received == False:
            self.bin_parts = BinPartsClass(msg)
            self.bin_parts_received = True

    # callback function: identify and store parts on Conveyor 
    def conveyer_parts_callback(self, msg:ConveyorParts):
        if self.conveyer_parts_received == False:
            self.conveyer_parts = ConveyerPartsClass(msg)
            self.conveyer_parts_received = True
    
    # callback function: store objects of Incoming Orders
    def store_order_callback(self, msg:Order):
        self.get_logger().info("Incoming Order ==> ID: %s || Type: %s" % (msg.id, msg.type))
        self.orders.append(OrderClass(msg))
        if msg.type == 0 or msg.type == "0" : # if Kitting Order, check Insufficient Parts Challenge
            self.check_sufficient_order() 
                  
    # ============================================================== #
    # |------------| Insufficient Parts Methods |------------| #
    # ============================================================== #
    
    # helper function: stores part in Matrix DS
    def calc_total_parts(self):
        if self.calculated_total_parts == False:
            self.total_parts = np.zeros((5,4))  # 5 rows, 4 columns --> matrix DS, stores 1 or 0
            # loop through each bin and each part in the bin. add to DS
            for bin in self.bin_parts.bins:
                for part in bin.parts:
                    self.total_parts[part.part.part_color, part.part.part_type - 10] += part.quantity
            # loop through each part on conveyor belt. add to DS
            for part in self.conveyer_parts.parts:
                self.total_parts[part.part.part_color, part.part.part_type - 10] += part.quantity
            self.calculated_total_parts = True
            # print out everything stored in the DS --> Initial Total Parts
            self.get_logger().info(".......................")
            self.get_logger().info("Initial Total Parts")
            self.get_logger().info("color 4, type 11: %i" % (self.total_parts[4,1]))
            self.get_logger().info("color 2, type 11: %i" % (self.total_parts[2,1]))
            self.get_logger().info("color 2, type 12: %i" % (self.total_parts[2,2]))
            self.get_logger().info("color 3, type 13: %i" % (self.total_parts[3,3]))
            self.get_logger().info("color 3, type 10: %i" % (self.total_parts[3,0]))
            self.get_logger().info("color 1, type 13: %i" % (self.total_parts[1,3]))
            self.get_logger().info("color 0, type 12: %i" % (self.total_parts[0,2]))
            self.get_logger().info(".......................")

    # helper function: checks if sufficeint parts are present to complete order
    def check_sufficient_order(self):
        order_parts = np.zeros((5,4)) # create matrix from storing order-requirements
        for part in self.orders[-1].task.parts:
            order_parts[part.part_color, part.part_type - 10] += 1 
        self.order_parts_array.append(order_parts)
        remaining_parts = self.total_parts - order_parts     
        # outcomes --> 0: part exists || >0: part not needed. part || <0: insufficient parts
        if np.any(remaining_parts < 0):  
            self.get_logger().info(".......................")
            self.get_logger().info("insufficient parts")
            insufficient_parts = np.where(remaining_parts<0)
            # report missing parts
            missing_part_no = np.shape(insufficient_parts)[1]
            for i in range(missing_part_no):
                self.get_logger().info("missing color %i, type %i" % (insufficient_parts[0][i],  insufficient_parts[1][i]+10))
            self.get_logger().info(".......................")
        else:
            # update total parts after deducted from order
            self.update_total_parts()

    # helper function: update Avaialble parts List
    def update_total_parts(self):
        self.total_parts = self.total_parts - self.order_parts_array[-1]
        self.get_logger().info(".......................")
        self.get_logger().info("Remaining Total Parts")
        self.get_logger().info("color 4, type 11: %i" % (self.total_parts[4,1]))
        self.get_logger().info("color 2, type 11: %i" % (self.total_parts[2,1]))
        self.get_logger().info("color 2, type 12: %i" % (self.total_parts[2,2]))
        self.get_logger().info("color 3, type 13: %i" % (self.total_parts[3,3]))
        self.get_logger().info("color 3, type 10: %i" % (self.total_parts[3,0]))
        self.get_logger().info("color 1, type 13: %i" % (self.total_parts[1,3]))
        self.get_logger().info("color 0, type 12: %i" % (self.total_parts[0,2]))
        self.get_logger().info(".......................")

    # ============================================== #
    # |------------| Helper Methods |------------| #
    # ============================================== #
    
    # def performTask(elf, msg:CompetitionState):
    #     if msg.competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
    #         self.sort_order()
    #         self.executeTask([LIST of Orders])
    #         flag_change
            
    # def execiteTask():
    
    #  callback function: checks order announcement, sort orders, and submit orders [Make it Dynamic]
    def competition_state_callback(self, msg:CompetitionState):
        if self.bin_parts_received == True and self.conveyer_parts_received == True:
            self.calc_total_parts()
        if msg.competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE and self.submitted == False:
            self.get_logger().info("-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x")
            self.sort_order()
            self.performTask()
            self.submit_order()
            self.submitted = True
        self.submit_state_publisher.update_data(self.submit_state)
        rclpy.spin_once(self.submit_state_publisher)
            
    def performTask(self):
        # self.total_parts  --- contains Available Parts
        # self.orders -- contains tasks to perform
        self.get_logger().info(" Performing tasks")
        for order_obj in self.orders:
            if order_obj.type == 0:
                self.get_logger().info("================================================")
                kitting_order = floorRobotClass()
                # self.get_logger().info("%s" % kitting_order.checkInSPartChallenge())
                self.get_logger().info("%s" % kitting_order.pickPart(self.conveyer_parts.parts))
                self.get_logger().info("%s" % kitting_order.placePart())
                self.get_logger().info("%s" % kitting_order.pickTray())
                self.get_logger().info("%s" % kitting_order.placeTray())
                self.get_logger().info("%s" % kitting_order.checkAGVTray())
                self.get_logger().info("%s" % kitting_order.actionAGV())
                self.get_logger().info("================================================")

        return None
        
    # helper function: sort order in terms of priority [convert it to Dynamic Queue]
    # sort and retain the Order Objects
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
    