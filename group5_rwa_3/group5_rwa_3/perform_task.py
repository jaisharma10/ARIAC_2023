class FloorRobotClass:
    def __init__(self):
        self.current_tool = "part"
        
    def PickPart(self, part_type,ConvPartCondition=False,BinPartCondition=False):
        if ConvPartCondition==True:    
            return("[FloorRobot] pick {} from conveyor".format(part_type))
        elif BinPartCondition==True:
             return("[FloorRobot] pick {} from Bin".format(part_type))    
    
    def PlacePart(self, part_type, bin_slot=None, quadrant=None):
        if bin_slot is not None:
            return("[FloorRobot] place {} in bin {} - slot {}".format(part_type, bin_slot[0], bin_slot[1]))
        elif quadrant is not None:
            return("[FloorRobot] place {} in quadrant {}".format(part_type, quadrant))
    
    def ChangeTool(self, tool_type):
        if tool_type == "part":
            return("[FloorRobot] change to part gripper")
        elif tool_type == "tray":
            return("[FloorRobot] change to tray gripper")
    
    def PickTray(self, tray_id):
        return("[FloorRobot] pick tray id {}".format(tray_id))
    
    def PlaceTray(self, agv_id):
        return("[FloorRobot] place tray on agv {}".format(agv_id))
        
class CeilingRobotClassKitting:

    def PickPart(self, part_type, bin_slot):
        return("[CeilingRobot] pick {} from bin {} - slot {}".format(part_type, bin_slot[0], bin_slot[1]))
    
    def PlacePart(self, part_type, quadrant):
        return("[CeilingRobot] place {} in quadrant {}".format(part_type, quadrant))
        
    def LockAgv(self, agv_id):
        return("lock agv {}".format(agv_id))
        
    def MoveAgv(self, agv_id, destination):
        return("move agv {} to {}".format(agv_id, destination))
    
class CeilingRobotClassAssembly:      
      
    def PickPart(self, part_type, agv):
        return("[CeilingRobot] pick {} from agv {} ".format(part_type,agv))
    
    def PlacePart(self, part_type, station):
        return("[CeilingRobot] place {} on station {}".format(part_type, station))
        
    def LockAgv(self, agv_id):
        return("lock agv {}".format(agv_id))
        
    def MoveAgv(self, agv_id, destination):
        return("move agv {} to {}".format(agv_id, destination))


