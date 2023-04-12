#!/usr/bin/env python3

__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'

"""
Holds Classes Relevant to sensors and cameras
"""

##### Part Classes #####

# class PositionClass():
#     """ Stores Pose -- Position
#     """
#     def __init__(self, msg):
#         self.x = msg.x
#         self.y = msg.y 
#         self.z = msg.z  

# class OrientationClass():
#     """ Stores Pose - Orientation
#     """
#     def __init__(self, msg):
#         super().__init__(msg.pt)
#         self.x = msg.x
#         self.y = msg.y
#         self.z = msg.z
#         self.w = msg.w

class PartClass():
    """ Stores Part Colour and Part Type
    """
    def __init__(self, msg):
        self.part_color = msg.color
        self.part_type = msg.type 
 
class alcPartClass(PartClass): 
    """ Inherit PartClass 
    """
    def __init__(self, msg):
        super().__init__(msg.part)
        self.pose = msg.pose
        # self.x = msg.pose.position.x
        # self.y = msg.pose.position.y
        
        
class alcTrayClass():
    """ Store Tray Id and Tray Pose 
    """
    def __init__(self, msg, side = None):
        self.tray_id = msg.id
        self.side = side
        self.tray_pose = msg.pose
    
