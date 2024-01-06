cmake_minimum_required(VERSION 3.0.2)
project(bug_alg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
)

catkin_python_setup()

add_executable(go_to_point scripts/go_to_point.cpp)
add_executable(wall_follower scripts/follow_wall.cpp)
add_executable(wall_follower_left scripts/follow_wall_left.cpp)
add_executable(bug1 scripts/bug1.cpp)

target_link_libraries(go_to_point ${catkin_LIBRARIES})
target_link_libraries(wall_follower ${catkin_LIBRARIES})
target_link_libraries(wall_follower_left ${catkin_LIBRARIES})
target_link_libraries(bug1 ${catkin_LIBRARIES})

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################




###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros
  DEPENDS GAZEBO
)
###########
## Build ##
###########

include_directories(
link_directories(${GAZEBO_LIBRARY_DIRS})
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################


