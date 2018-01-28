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
  sensor_msgs
)

## Find Boost
find_package(Boost REQUIRED COMPONENTS 
  system
  filesystem
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

# YAML library
find_library(YAML_CPP_LIBRARIES yaml-cpp)
if(NOT YAML_CPP_LIBRARIES)
  # If yaml-cpp not found in the system, try finding it as a user CMake-generated project
  find_package(yaml-cpp REQUIRED)
  include_directories(${YAML_CPP_INCLUDE_DIRS})
endif(NOT YAML_CPP_LIBRARIES)


## Add actions
add_action_files(DIRECTORY action FILES 
  MoveArm.action
  FindEdge.action
)

## Add messages
add_message_files(
 FILES
 ObjectFeature.msg
 StopCondition.msg
)

add_service_files(
  FILES
  DetectObjectFeature.srv
  GetTaskSpec.srv
  GetMotionSpec.srv
)

## Generate messages
generate_messages(
  DEPENDENCIES 
  std_msgs 
  actionlib_msgs
  geometry_msgs
  sensor_msgs
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

## Constraint controller for PR2 pr2
add_executable(constraint_controller_pr2 
  src/constraint_controller_pr2.cpp
  src/giskard_adapter.cpp
)
target_link_libraries(constraint_controller_pr2 ${catkin_LIBRARIES})
add_dependencies(constraint_controller_pr2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Constraint controller for gazebo free_ees
add_executable(constraint_controller_free_ees 
  src/constraint_controller_free_ees.cpp
  src/giskard_adapter.cpp
)
target_link_libraries(constraint_controller_free_ees ${catkin_LIBRARIES})
add_dependencies(constraint_controller_free_ees ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Task executive
add_executable(task_executive 
  src/task_executive.cpp 
  src/twist_log.cpp
  src/task.cpp
  src/experiment.cpp
  src/spec.cpp
)
target_link_libraries(task_executive ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(task_executive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Task executive
add_executable(task_executive2 
  src/task_executive2.cpp 
  src/twist_log.cpp
)
target_link_libraries(task_executive2 ${catkin_LIBRARIES})
add_dependencies(task_executive2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Edge detector
add_executable(edge_detector 
  src/edge_detector.cpp
)
target_link_libraries(edge_detector ${catkin_LIBRARIES})
add_dependencies(edge_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Knowledge manager
add_executable(knowledge_manager 
  src/knowledge_manager.cpp 
)
target_link_libraries(knowledge_manager ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(knowledge_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Feature detector
add_executable(feature_detector 
  src/feature_detector.cpp
)
target_link_libraries(feature_detector ${catkin_LIBRARIES})
add_dependencies(feature_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Gazebo Plugins

## Force controller plugin
add_library(velocity_controller_plugin plugins/velocity_controller_plugin.cpp)
target_link_libraries(velocity_controller_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Force controller plugin
add_library(position_controller_plugin plugins/position_controller_plugin.cpp)
target_link_libraries(position_controller_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## TF broadcaster plugin
add_library(tf_broadcaster_plugin plugins/tf_broadcaster_plugin.cpp)
target_link_libraries(tf_broadcaster_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Grip plugin
add_library(GripPlugin plugins/GripPlugin.cc)
target_link_libraries(GripPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Stick plugin
add_library(StickPlugin plugins/StickPlugin.cc)
target_link_libraries(StickPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Grip plugin
add_library(GrainsFactoryPlugin plugins/GrainsFactoryPlugin.cc)
target_link_libraries(GrainsFactoryPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Grip plugin
add_library(LasagnaFactoryPlugin plugins/LasagnaFactoryPlugin.cc)
target_link_libraries(LasagnaFactoryPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

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

