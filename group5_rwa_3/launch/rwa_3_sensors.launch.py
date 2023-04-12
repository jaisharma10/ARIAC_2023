# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node

# this function is needed
def generate_launch_description():

    ld = LaunchDescription() # instantiate a Launchdescription object
    
    comp_state_subscriber_node = Node( # declare your Node
        package="group5_rwa_3", # package name
        executable="comp_state_subscriber.py" # executable as set in setup.py
    )
    sensor_subscriber_node = Node(
        package="group5_rwa_3",
        executable="sensor_subscriber.py"
    )
    
    ld.add_action(sensor_subscriber_node)  # add each Node to the LaunchDescription object
    ld.add_action(comp_state_subscriber_node)   # add each Node to the LaunchDescription object
    return ld # return the LaunchDescription object     
