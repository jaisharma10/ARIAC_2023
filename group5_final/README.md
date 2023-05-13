
# **Documentation**

Instructions:

- Documetation can be found in etc folder.
- Open the file on Path with any Internet Browser 
- *Path*: group5_rwa_3/etc/docs_output/group5_rwa_3/index.html


## RWA 3 Submission

Start the ARIAC environment

```
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=group5_rwa_3 sensor_config:=group5_sensors trial_name:=rwa3
```

Start Move It 2

```
    ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py
```

Start Task Manager Node and Floor Robot Node using RWA 3 launch file 

```
    ros2 launch group5_rwa_3 rwa_3.launch.py
```
## RWA 4 Submission

Start the ARIAC environment

```
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=group5_rwa_3 sensor_config:=group5_sensors trial_name:=rwa4
```

Start Move It 2

```
    ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py
```

Start Task Manager Node and Floor Robot Node using RWA 3 launch file 

```
    ros2 launch group5_rwa_3 rwa_3.launch.py
```

## Dependencies

- Ubuntu 20.04
- ROS2 - Galactic Geochelone
- Gazebo
- MoveIt 2
- ARIAC 2023
 
## Team Members

* [Jai Sharma](https://github.com/SaumilShah66)
* [Varith Punturaumporn](https://github.com/varithpu)
* [Shubham Takbhate](https://github.com/Shubhamtakbhate1998)
* [Pratik Patel](https://github.com/pratik2394)
 
