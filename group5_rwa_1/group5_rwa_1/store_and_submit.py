#!/usr/bin/env python3

__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'

"""
Holds Classes Relevant 
    - Task Type: 
        - Kitting
        - Assembly
        - Combined Orders
    - Parts 
"""

##### Part Classes #####

class PartClass():
    """This is a Parent class. Since, each of the order has attribute 
       color and type we made this as parent class
    """
    def __init__(self, msg):
        self.part_color = msg.color
        self.part_type = msg.type 
 
class AssemblyPartClass(PartClass):
    """ Inherit PartClass and create a new class for Assembly order, 
        The same class can be reused for the Combined order
    """
    def __init__(self, msg):
        super().__init__(msg.part)
        self.pose = msg.assembled_pose
        self.direction = msg.install_direction

class KittingPartClass(PartClass):
    """ Inherit PartClass and create a new class for Kitting order, 
    """
    def __init__(self, msg):
        super().__init__(msg.part)
        self.quadrant = msg.quadrant 

##### Task Classes #####

class AssemblyTaskClass():  
    """ Class For Assembly Tasks"""
    def __init__(self, msg):
        self.agv_numbers = msg.agv_numbers
        self.station = msg.station
        self.number_of_parts = len(msg.parts)
        self.parts = []
        for i in range(self.number_of_parts):
            self.parts.append(AssemblyPartClass(msg.parts[i]))

class KittingTaskClass():
    """ Class For Kitting Tasks"""
    def __init__(self, msg):
        self.agv_number = msg.agv_number
        self.tray_id = msg.tray_id
        self.destination = msg.destination
        self.number_of_parts = len(msg.parts)
        self.parts = []
        for i in range(self.number_of_parts):
            self.parts.append(KittingPartClass(msg.parts[i]))

class CombinedTaskClass(): 
    """ Class For Combined Tasks"""
    def __init__(self, msg):
        self.station = msg.station
        self.number_of_parts = len(msg.parts)
        self.parts = []
        for i in range(self.number_of_parts):
            self.parts.append(AssemblyPartClass(msg.parts[i]))

##### Order Classes #####

class OrderClass():
    """ Order class checks the order type and creates the instances for each order type"""
    def __init__(self, msg):
        self.id = msg.id
        self.type = msg.type
        self.priority = msg.priority
        if self.type == msg.KITTING:
            self.task = KittingTaskClass(msg.kitting_task)
        elif self.type == msg.ASSEMBLY:
            self.task = AssemblyTaskClass(msg.assembly_task)
        elif self.type == msg.COMBINED:
            self.task = CombinedTaskClass(msg.combined_task)
        else:
            pass
