# import rclpy
# from rclpy.node import Node

#Ignore 
# class floorRobotClass():
    
#     def __init__(self):
#         self.x = None
#         self.ConvPart=[]  
#         self.PartColor=dict([(0, "Red"), (1, "Green"),(2,"Blue"),(3,"Orange"),(4,"Purple")])
#         self.PartType=dict([(10, "Battery"), (11, "Pump"),(12,"Sensor"),(13,"Regulator")])
#     def checkInSPartChallenge(self):
#         return "Check if parts are Sufficeint for the Order"

#     def pickPart(self,conveyorParts):
#         # if part on conveyor --> destination: empty bin
#             # return ()
#         # if part on bin --> destination is tray
#             # return ()
#         self.conveyorParts=conveyorParts
#         str1=" [Floor Robot] Pick up "
#         for part in conveyorParts:
#             str2=str1+str(self.PartColor[part.part.part_color])+str(" ")+str(self.PartType[part.part.part_type])+str(" ")
#             self.ConvPart.append(str2)
#             str1=" [Floor Robot] Pick up "
     
#         return (self.ConvPart) 
        
            
            
#         # return("Return Part and Destination")
        
#     def placePart(self):
#         self.ConvPart=[]
#         str1=" [Floor Robot] Place "
#         for part in self.conveyorParts:
#             str2=str1+str(self.PartColor[part.part.part_color])+str(" ")+str(self.PartType[part.part.part_type])+str(" ") +"to empty bin"
#             self.ConvPart.append(str2)
#             str1=" [Floor Robot] Place "
        
#         return(self.ConvPart)
        
#     def pickTray(self):
#         # input trayID --> get position
#         return("pick up tray")
    
#     def placeTray(self):
#         # input AGV # --> get position
#         # call changeGripper when tray is placed
#         return("place tray on AGV")
            
#     def changeGripper(self): 
#         # toggle function
#         print("Swap Tray Gripper and Part Gripper")
    
#     def checkAGVTray(self):
#         # checks if there is a tray on AGV
#         # input AGV number,tray ID
#         # if not --> checkGripperStatus --> change Gripper if needed [call self.changeGripper()]
#         # if not:
#         #      --> Pick and Place TRAY
#         return("Check if Tray on AGV. Check/Change Gripper Status. Pick/Place tray if needed.")
    
#     @staticmethod
#     def actionAGV():
#         print("Lock AGV")
#         return("Move AGV to warehouse")


# # based on msg.type of order
# # if KITTING

# # kitting_order = floorRobotClass()

# # kitting_order.checkInSPartChallenge()
# # # conveyor parts are moved
# # kitting_order.pickPart()
# # kitting_order.placePart()

# # # ensure correct TRAY is on correct AGV
# # kitting_order.checkAGVTray()

# # # bin parts are moved to tray quadrants 
# # kitting_order.pickPart()
# # kitting_order.placePart()
# # kitting_order.actionAGV()

# # Submit orde

class FloorRobotClass:
    def __init__(self):
        self.current_tool = "part"
        
    def PickPart(self, part_type,ConvPartCondition=False,BinPartCondition=False):
        if ConvPartCondition==True:    
            print("[FloorRobot] pick {} from conveyor".format(part_type))
        elif BinPartCondition==True:
             print("[FloorRobot] pick {} from Bin".format(part_type))    
    
    def PlacePart(self, part_type, bin_slot=None, quadrant=None):
        if bin_slot is not None:
            print("[FloorRobot] place {} in bin {} - slot {}".format(part_type, bin_slot[0], bin_slot[1]))
        elif quadrant is not None:
            print("[FloorRobot] place {} in quadrant {}".format(part_type, quadrant))
    
    def ChangeTool(self, tool_type):
        if tool_type == "part":
            print("[FloorRobot] change to part gripper")
        elif tool_type == "tray":
            print("[FloorRobot] change to tray gripper")
    
    def PickTray(self, tray_id):
        print("[FloorRobot] pick tray id {}".format(tray_id))
    
    def PlaceTray(self, agv_id):
        print("[FloorRobot] place tray on agv {}".format(agv_id))
        
class CeilingRobotClass:
    def __init__(self):
        pass
    
    def PickPart(self, part_type, bin_slot):
        print("[CeilingRobot] pick {} from bin {} - slot {}".format(part_type, bin_slot[0], bin_slot[1]))
    
    def PlacePart(self, part_type, quadrant):
        print("[CeilingRobot] place {} in quadrant {}".format(part_type, quadrant))
        
    def LockAgv(self, agv_id):
        print("lock agv {}".format(agv_id))
        
    def MoveAgv(self, agv_id, destination):
        print("move agv {} to {}".format(agv_id, destination))
# floor_robot = FloorRobotClass()
# ceiling_robot = CeilingRobotClass()
#Example for running the class
# pick and place green regulator
# floor_robot.PickPart()
# floor_robot.PlacePart("green regulator", bin_slot=(8, 1))

# # pick and place red sensor
# floor_robot.PickPart("red sensor")
# floor_robot.PlacePart("red sensor", bin_slot=(8, 2))

# # switch to tray gripper and pick tray
# floor_robot.ChangeTool("tray")
# floor_robot.PickTray(3)

# # place tray on AGV
# floor_robot.PlaceTray(4)

# # pick and place green regulator
# ceiling_robot.PickPart("green regulator", bin_slot=(8, 1))
# ceiling_robot.PlacePart("green regulator", quadrant=2)

# # switch to part gripper and pick purple pump
# floor_robot.ChangeTool("part")
# floor_robot.PickPart("purple pump")
# floor_robot.PlacePart("purple pump", quadrant=1)

# # pick and place red sensor
# floor_robot.PickPart("red sensor")
# floor_robot.PlacePart("red sensor", bin_slot=(8, 2))

# # lock AGV and move to warehouse
# ceiling_robot.LockAgv(4)
# ceiling_robot.MoveAgv(4,"warehous")

