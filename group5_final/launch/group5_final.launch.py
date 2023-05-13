#!/usr/bin/env python3

__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'

'''
The Launch File starts the following nodes:
    - task manager node
    - competition state node
    - floor robot node
'''

import os
import yaml

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    ld = LaunchDescription() # instantiate a Launchdescription object
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/ariac_robots", "ariac_robots.urdf.xacro"]), 
            " "
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {"robot_description_semantic": load_file("ariac_moveit_config", "srdf/ariac_robots.srdf")}
    robot_description_kinematics = {"robot_description_kinematics": load_yaml("ariac_moveit_config", "config/kinematics.yaml")}

    competition_state_node = Node( # declare your Node
        package="group5_final",    # package name
        executable="competition_state_node.py" # executable as set in setup.py
    )
    
    task_manager_node = Node(
        package="group5_final",
        executable="task_manager_node.py"
    )
        
    floor_robot_node = Node(
        package="group5_final",
        executable="robot_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )
    
    # RVIZ 
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("group5_final"), "rviz", "group5.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
        arguments=['-d', rviz_config_file]
    )

    # add nodes that you want to launch
    ld.add_action(task_manager_node)  
    ld.add_action(competition_state_node)   
    ld.add_action(floor_robot_node)
    ld.add_action(rviz)

    return ld # return the LaunchDescription object     



