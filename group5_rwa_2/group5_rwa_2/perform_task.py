import rclpy
from rclpy.node import Node


class floorRobotClass():
    
    def __init__(self):
        self.x = None
        
    def checkInSPartChallenge(self):
        return "Check if parts are Sufficeint for the Order"

    def pickPart(self,conveyorParts):
        # if part on conveyor --> destination: empty bin
            # return ()
        # if part on bin --> destination is tray
            # return ()
        str1="p"  
        for part in conveyorParts:
            str1=str1+str(part.part.part_type)
        return(str1)    
            
            
        return("Return Part and Destination")
        
    def placePart(self):
        # input is part and destination
        return("place part in destination")
        
    def pickTray(self):
        # input trayID --> get position
        return("pick up tray")
    
    def placeTray(self):
        # input AGV # --> get position
        # call changeGripper when tray is placed
        return("place tray on AGV")
            
    def changeGripper(self): 
        # toggle function
        print("Swap Tray Gripper and Part Gripper")
    
    def checkAGVTray(self):
        # checks if there is a tray on AGV
        # input AGV number,tray ID
        # if not --> checkGripperStatus --> change Gripper if needed [call self.changeGripper()]
        # if not:
        #      --> Pick and Place TRAY
        return("Check if Tray on AGV. Check/Change Gripper Status. Pick/Place tray if needed.")
    
    @staticmethod
    def actionAGV():
        print("Lock AGV")
        return("Move AGV to warehouse")


# based on msg.type of order
# if KITTING

# kitting_order = floorRobotClass()

# kitting_order.checkInSPartChallenge()
# # conveyor parts are moved
# kitting_order.pickPart()
# kitting_order.placePart()

# # ensure correct TRAY is on correct AGV
# kitting_order.checkAGVTray()

# # bin parts are moved to tray quadrants 
# kitting_order.pickPart()
# kitting_order.placePart()
# kitting_order.actionAGV()

# Submit order
