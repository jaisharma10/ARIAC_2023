uint8 KITTING=0
uint8 ASSEMBLY=1
uint8 COMBINED=2

ariac_msgs/PartPose[] partscombined
string id
string step
uint8 type # KITTING, ASSEMBLY, or COMBINED
bool priority
ariac_msgs/KittingTask kitting_task 
ariac_msgs/AssemblyTask assembly_task
ariac_msgs/CombinedTask combined_task