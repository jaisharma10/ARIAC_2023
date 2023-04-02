#!/usr/bin/env python3

import rclpy
import numpy as np
import heapq as hq
import time
from rclpy.node import Node

from ariac_msgs.msg import Order, BinParts, ConveyorParts

from group5_rwa_2.store_and_submit import OrderClass
from group5_rwa_2.part_locations import ConveyerPartsClass, BinPartsClass
from group5_rwa_2.submit_order_nodes import SubmitOrderClient, SubmitStatePublisher
from group5_rwa_2.perform_task import FloorRobotClass, CeilingRobotClassKitting,CeilingRobotClassAssembly
from group5_rwa_2.agv_apis import ManageAGV


class OrderSubscriber(Node):
    """
    Subscriber class for subscribing orders
    """
    # initialization, create all publisher and subscribers
    def __init__(self):
        super().__init__('order_subscriber')
     
        self.orders = []
        hq.heapify(self.orders)           # build a heapq to store incoming orders
        self.order_parts_array = []
 
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
        hq.heappush(self.orders, (0 if msg.priority else 1, OrderClass(msg)))       
        if self.orders:                                                             # if heapq list is not empty
            self.pop_order = hq.heappop(self.orders)                                # pop the highest priority order
            if self.pop_order[1].type == 0:                        # if Kitting Order: check Insufficient Parts Challenge, then perform 
                self.check_sufficient_order()                                   
                self.performTask(self.pop_order[1])          
            else:                                                  # else: perform Assembly or Combined order directly
                self.performTask(self.pop_order[1])                              
            
    # callback function: identify and store parts in Bins
    def bin_parts_callback(self, msg:BinParts) -> None:
        if self.bin_parts_received == False:
            self.bin_parts = BinPartsClass(msg)
            self.bin_parts_received = True

    # callback function: identify and store parts from Conveyor 
    def conveyer_parts_callback(self, msg:ConveyorParts) -> None:
        if self.conveyer_parts_received == False:
            self.conveyer_parts = ConveyerPartsClass(msg)
            self.conveyer_parts_received = True
    
    # ============================================================== #
    # |---------------| Insufficient Parts Methods |---------------| #
    # ============================================================== #
    
    # helper function: stores parts in Matrix DataStructure
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
            # self.get_logger().info out everything stored in the DS --> Initial Total Parts
            self.get_logger().info("================================================================================")
            self.get_logger().info("Initial Available Parts")
            out_arr=np.argwhere(self.total_parts)
           
            for j, k in out_arr:  # using j,k instead of saving new variables to memory
                str2 = str("Quantity % i" % self.total_parts[j,k] )
                str1 = ("Color is: % s" % self.PartColor[j] ) + (" ") + ("Type is: % s" % self.PartType2[k]) + (" ") + ("Part % s" % str2)
                self.get_logger().info(str1)
                
            self.get_logger().info("================================================================================")

    # helper function: checks if sufficeint parts are present to complete kitting order
    def check_sufficient_order(self):
        order_parts = np.zeros((5,4)) # create matrix from storing order-requirements
        for part in self.pop_order[1].task.parts:            
            order_parts[part.part_color, part.part_type - 10] += 1 
        self.order_parts_array.append(order_parts)
        remaining_parts = self.total_parts - order_parts     
        # outcomes --> 0: part exists || >0: part not needed. part || <0: insufficient parts
        if np.any(remaining_parts < 0):  
            self.get_logger().warn("Insufficient parts to complete Kitting order")
            insufficient_parts = np.where(remaining_parts<0)
            # report missing parts
            missing_part_no = np.shape(insufficient_parts)[1]
            for i in range(missing_part_no):
                self.get_logger().info("    - missing color %s, type %s" % (self.PartColor[insufficient_parts[0][i]],  self.PartType[insufficient_parts[1][i]+10]))
        
    # ============================================== #
    # |----------| Perform Task Methods |----------| #
    # ============================================== #
    
    def performTask(self, current_order):
        
        self.get_logger().info("===========================================================")
        order_obj = current_order
        self.get_logger().info("Executing Order ==> ID: %s " % (order_obj.id))
        
        # Execute Kitting Order Task
        if order_obj.type==0: 
            self.get_logger().info("  --> Working on Kitting Order")

            floor_robot = FloorRobotClass()
            # Pick and Place each part on Conveyor
            for  p in self.conveyer_parts.parts:
                TypeofPart = self.PartType[p.part.part_type] 
                ColoroffPart = self.PartColor[p.part.part_color]
                self.get_logger().info(floor_robot.PickPart(ColoroffPart + TypeofPart, ConvPartCondition=True))
                self.get_logger().info(floor_robot.PlacePart(ColoroffPart + TypeofPart, bin_slot=(8, 1)))

            self.get_logger().info(" -------------------------------------------- ")
            self.get_logger().info(floor_robot.ChangeTool("tray"))                      # Change Floor Robot Tool to 'TRAY'
            self.get_logger().info(floor_robot.PickTray(order_obj.task.tray_id))        # Pick Tray
            self.get_logger().info(floor_robot.PlaceTray(order_obj.task.agv_number))    # Place Tray
            
            self.get_logger().info(" -------------------------------------------- ")
            # minimal_client = ManageAGV(order_obj.task.agv_number)                       # Move AGVs ?
            # minimal_client.move_agv_to(order_obj.task.destination)
            # minimal_client.get_logger().info( 'Moving AGV %i to location %i' % (order_obj.task.agv_number, order_obj.task.destination))
            # minimal_client.destroy_node()
              
            celing_robot=CeilingRobotClassKitting()            
            self.get_logger().info(" -------------------------------------------- ")
            
            # is this for BINs to TRAYs pick and place?
            for p in (order_obj.task.parts):
                if self.total_parts[p.part_color,p.part_type-10] > 0:
                    self.total_parts[p.part_color,p.part_type-10] = self.total_parts[p.part_color,p.part_type-10] - 1
                    TypeofPart = self.PartType[p.part_type] + " "
                    ColoroffPart = self.PartColor[p.part_color] + " "
                    self.get_logger().info(celing_robot.PickPart(ColoroffPart+TypeofPart, bin_slot=(8,1)))
                    self.get_logger().info(celing_robot.PlacePart(ColoroffPart+TypeofPart, p.quadrant))

            self.get_logger().info(" -------------------------------------------- ")
            self.get_logger().info(celing_robot.LockAgv(order_obj.task.agv_number))
            self.get_logger().info(celing_robot.MoveAgv(order_obj.task.agv_number,"warehouse") )
            
            self.get_logger().info(" -------------------------------------------- ")
            
            # instead of repeating, make a function --> def print_parts(___):
            out_arr=np.argwhere(self.total_parts)
            for j, k in out_arr:  
                str2 = str("Quantity % i" % self.total_parts[j,k] )
                str1 = ("Color is: % s" % self.PartColor[j] ) + (" ") + ("Type is: % s" % self.PartType2[k]) + (" ") + ("Part % s" % str2)
                self.get_logger().info(str1)
                            
        # Execute Assembly Order Task     
        if order_obj.type==1: 
            self.get_logger().info("  --> Working on Assembly Order using Ceiling Robot")
            
            celing_robot = CeilingRobotClassAssembly()
            for agv_num in order_obj.task.agv_numbers:
                celing_robot.LockAgv(agv_num)
                self.get_logger().info(celing_robot.MoveAgv(agv_num, order_obj.task.station))
                
                # what does this do? is moving agvs repeatable?
                # minimal_client = ManageAGV(agv_num)
                # minimal_client.move_agv_to(order_obj.task.station)
                # minimal_client.get_logger().info('Moving AGV %i to location %i' % (agv_num, order_obj.task.station))
                # minimal_client.destroy_node()
            
            self.get_logger().info(" -------------------------------------------- ")
            for order_parts in (order_obj.task.parts):
                TypeofPart = self.PartType[order_parts.part_type] 
                ColoroffPart = self.PartColor[order_parts.part_color] + " "
                self.get_logger().info(celing_robot.PickPart(ColoroffPart + TypeofPart, "agv1"))
                self.get_logger().info(celing_robot.PlacePart(ColoroffPart + TypeofPart,order_obj.task.station))
            
        # Execute Combined Order Task     
        if order_obj.type==2: 
            
            self.get_logger().info("  --> Working on Combined Order using Floor and Ceiling Robot") 
            self.get_logger().info("  --> Floor robot will pick up parts from conveyor belt")
            self.get_logger().info("  --> Ceiling robot will pick up parts from bins")

            """
            Based on station number, the parts needs to be placed on AGV1 or AGV2 or AGV3 or AGV4. 
            If the station number is AS1 or AS2, then any of the free AGV (1 or 2- based on status) can be used to place the parts.
            likewise if the station number is AS3 or AS4, then any of the free AGV (3 or 4- based on status) can be used to place the parts. 
            For now, instead of querying the status of AGV, we will use AGV1 for AS1 and AS2 and AGV3 for AS3 and AS4. 
            """
            
            if order_obj.task.station == 1 or order_obj.task.station == 2: 
                the_agv_number = 1
            if order_obj.task.station == 3 or order_obj.task.station == 4: 
                the_agv_number = 3 
                 
            floor_robot = FloorRobotClass()
                
            self.get_logger().info(" -------------------------------------------- ")
            self.get_logger().info(floor_robot.ChangeTool("tray"))
            self.get_logger().info(floor_robot.PickTray(the_agv_number))
            self.get_logger().info(floor_robot.PlaceTray(the_agv_number))
            
            self.get_logger().info(" -------------------------------------------- ")
            self.i = 1

            celing_robot = CeilingRobotClassKitting()            
            for p in (order_obj.task.parts):
                if self.total_parts[p.part_color,p.part_type-10] > 0 :
                    self.total_parts[p.part_color,p.part_type-10] = self.total_parts[p.part_color,p.part_type-10] - 1
                    TypeofPart = self.PartType[p.part_type] + " "
                    ColoroffPart = self.PartColor[p.part_color] + " "
                    self.get_logger().info(celing_robot.PickPart(ColoroffPart+TypeofPart,bin_slot=(8,1)))
                    self.get_logger().info(celing_robot.PlacePart(ColoroffPart+TypeofPart, self.i))
                    self.i += 1
          
            self.get_logger().info(" -------------------------------------------- ")
            self.get_logger().info("  --> Working on Assembly Part using Ceiling Robot")
            self.get_logger().info(f'  --> Moving AGV: {the_agv_number} to station: {order_obj.task.station}')
            
            celing_robot = CeilingRobotClassAssembly()
            self.get_logger().info(celing_robot.LockAgv(the_agv_number))
            self.get_logger().info(celing_robot.MoveAgv(the_agv_number,order_obj.task.station))

            self.get_logger().info(" -------------------------------------------- ")            
            for p in (order_obj.task.parts):
                TypeofPart=self.PartType[p.part_type] 
                ColoroffPart=self.PartColor[p.part_color] + " "
                self.get_logger().info(celing_robot.PickPart(ColoroffPart+TypeofPart,"agv1"))
                self.get_logger().info(celing_robot.PlacePart(ColoroffPart+TypeofPart,order_obj.task.station))

        self.get_logger().info("===========================================================")
     
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
    
