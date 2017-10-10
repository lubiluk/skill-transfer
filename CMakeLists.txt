cmake_minimum_required(VERSION 2.8.3)
project(skill_transfer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  actionlib 
  message_generation
  giskard_core
  giskard_ros_utils
  kdl_conversions
  std_msgs
  gazebo_msgs
  gazebo_ros
)

## Find Boost
find_package(Boost REQUIRED COMPONENTS 
  system
  filesystem
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

# YAML library
#find_library(YAML_CPP_LIBRARIES yaml-cpp)
#if(NOT YAML_CPP_LIBRARIES)
  # If yaml-cpp not found in the system, try finding it as a user CMake-generated project
  find_package(yaml-cpp REQUIRED)
  include_directories(${YAML_CPP_INCLUDE_DIRS})
#endif(NOT YAML_CPP_LIBRARIES)


## Add actions
add_action_files(DIRECTORY action FILES 
  MoveArm.action
)

## Add messages
add_message_files(
  FILES
  ControllerPoints.msg
)

## Generate messages
generate_messages(
  DEPENDENCIES 
  std_msgs 
  actionlib_msgs
  geometry_msgs
)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp actionlib
  DEPENDS Boost gazebo_ros
)

## Setup link dirs
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

## Setup include dirs
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Constraint controller
add_executable(constraint_controller 
  src/constraint_controller.cpp
)
target_link_libraries(constraint_controller ${catkin_LIBRARIES})
add_dependencies(constraint_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Task executive
add_executable(task_executive 
  src/task_executive.cpp 
  src/file_sequence.cpp
  src/twist_log.cpp
  src/task.cpp
)
target_link_libraries(task_executive ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(task_executive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Force controller Gazebo plugin
add_library(force_controller_plugin src/force_controller_plugin.cpp)
target_link_libraries(force_controller_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Controller visualization Gazebo plugin
#add_library(controller_visualization_plugin src/controller_visualization_plugin.cpp)
#target_link_libraries(controller_visualization_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Install scripts and executables
# install(PROGRAMS
#   scripts/gen_numbers.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(TARGETS averaging_server averaging_client
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

