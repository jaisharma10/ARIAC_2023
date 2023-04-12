# RWA 3 Submission

Through the assignment, accomplish some important tasks:
- 'List main features below'

## Build and Compile instructions

Start the ARIAC environment

```
    cd "your_colcon_workspace"/src
    git clone https://github.com/usnistgov/ARIAC.git
    git clone https://github.com/jaisharma10/ARIAC_2023/tree/master/group5_rwa_3
    cd ..
    colcon build
    source "your_colcon_workspace"/install/setup.bash
``````

## **Running Task Scripts**

Start the ARIAC environment

```
  ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=group5_rwa_3 sensor_config:=group5_sensors trial_name:=rwa3
```

Start the Sensor Testing Launch File

```
  ros2 launch group5_rwa_3 rwa_3_task_manager.launch.py 
```

## Team Memberss

* [Jai Sharma](https://github.com/jaisharma10/)
* [Varith Punturaumporn](https://github.com/varithpu)
* [Shubham Takbhate](https://github.com/Shubhamtakbhate1998)
* [Pratik Patel](https://github.com/pratik2394)


## Support

For any questions, email me at jaisharm@umd.edu

