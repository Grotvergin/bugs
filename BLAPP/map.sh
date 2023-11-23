#! /usr/bin/bash

#open main directory
cd ~/
#export robot by name
export TURTLEBOT3_MODEL=burger
source /opt/ros/noetic/setup.bash
path_="$(pwd)"
source $path_/catkin_ws/devel/setup.bash
#sleep is required for map to work properly
sleep 3
#launch a world 1 = world launch file
roslaunch turtlebot3_gazebo "$1"
exit 1
