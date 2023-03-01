
# ARIAC - Agile Robotics for Industrial Automation 2023

This project was developed as a part of ENPM663 Graduate Course - Building a Manufacturing Robot Software Systems at the University of Maryland, College Park.

The goal is to develop a robust execution software using ROS 2, C++ and Python. The simulation environment is a representation of an order fulfillment workcell. Kitting and assembly orders and retrieved from a server.  In a team, we program floor manipulators, ceiling manipulators and AGVs to execute the incoming orders. The code base must be robust enough to tackle Agility challenges like Faulty Parts, Sensor Blackout, Robot Malfunctions, Faulty Gripper etc.

![Environment](https://www.nist.gov/sites/default/files/styles/2800_x_2800_limit/public/images/2023/01/13/environment.png?itok=1FHdvs4c)

## Dependencies

- Ubuntu 20.04
- ROS2 - Galactic Geochelone
- Gazebo
- MoveIt 2
- ARIAC 2023

## ARIAC 2023

Find the latest resources here
  - Documentation [Link](https://ariac.readthedocs.io/en/latest/index.html)<br>
  - Github [Link](https://github.com/usnistgov/ARIAC)<br>
  - Installation [Link](https://ariac.readthedocs.io/en/latest/getting_started/installation.html)<br>


## Build and Compile instructions

Setup ARIAC environment and Group5 codebase

```
    cd "your_colcon_workspace"/src
    git clone https://github.com/usnistgov/ARIAC.git
    git clone https://github.com/jaisharma10/ARIAC_2023
    cd ..
    colcon build
    source "your_colcon_workspace"/install/setup.bash
```
 

## RWA 1 Submission

Through the assignment, accomplish 4 important tasks:
- Start Competition.
- Retrieve and store incoming orders as a Class object.
- Submit orders in the order of priority.
- End Competition.

### **Running Task Scripts**

Start the ARIAC environment

```
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa1
```

Start the complete Launch File

```
  ros2 launch group5 rwa_1.launch.py 
```

## RWA 2 Submission

*In Progress*

## Team Members

* [Jai Sharma](https://github.com/SaumilShah66)
* [Varith Punturaumporn](https://github.com/varithpu)
* [Shubham Takbhate](https://github.com/Shubhamtakbhate1998)
* Pratik Patel


## Support

For any questions, email me at jaisharm@umd.edu