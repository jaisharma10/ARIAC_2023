cmake_minimum_required(VERSION 3.8)
project(group5_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# more dependencies [Sensor and Floor Robot]
set(SENSOR_CAMERA_DEPENDENCIES
  std_srvs
  geometry_msgs
  tf2  
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
)

set(ROBOT_DEPENDENCIES
  moveit_ros_planning_interface
  group5_msgs
  shape_msgs
)

foreach(Dependency IN ITEMS ${SENSOR_CAMERA_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${ROBOT_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add cpp "include" directory
include_directories(include)
include_directories(include/group5_final)
include_directories(include/group5_msgs)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/competition_state_node.py
  nodes/task_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ executables
add_executable(robot_node src/robot.cpp)
add_executable(robot_v1_node src/robot_v1.cpp)
ament_target_dependencies(robot_node ${ROBOT_DEPENDENCIES})
ament_target_dependencies(robot_v1_node ${ROBOT_DEPENDENCIES})

install(TARGETS
  robot_node
  robot_v1_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
        group5_final launch config include rviz DESTINATION 
        share/${PROJECT_NAME}
)

ament_package()
