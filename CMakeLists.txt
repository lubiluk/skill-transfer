cmake_minimum_required(VERSION 2.8.3)
project(skill_transfer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  std_msgs
  giskard_core
  giskard_ros_utils
  kdl_conversions
  visualization_msgs
  actionlib_msgs
  actionlib
  message_generation
  )

add_action_files(DIRECTORY action FILES MoveArm.action Averaging.action)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs gazebo_msgs visualization_msgs)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES skill_transfer
  #  CATKIN_DEPENDS roscpp std_msgs
  #  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/skill_transfer.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(task_executive src/task_executive.cpp)
add_dependencies(task_executive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_executive ${catkin_LIBRARIES})

add_executable(constraint_controller src/constraint_controller.cpp)
add_dependencies(constraint_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(constraint_controller ${catkin_LIBRARIES})
