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
)

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

add_executable(test_gazebo_comm src/test_gazebo_comm.cpp)
add_dependencies(test_gazebo_comm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_gazebo_comm ${catkin_LIBRARIES})

add_executable(task_executive src/task_executive.cpp)
target_link_libraries(task_executive ${catkin_LIBRARIES})

