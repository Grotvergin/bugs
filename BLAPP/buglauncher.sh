#! /usr/bin/bash

#get to main diectory
cd ~/
#launch bug algorithm/export robot model
export TURTLEBOT3_MODEL=burger
source /opt/ros/noetic/setup.bash
path_="$(pwd)"
source $path_/catkin_ws/devel/setup.bash
#launch an algorithm 1=name 2=x 3=y
roslaunch bug_alg "$1".launch des_x:="$2" des_y:="$3"
exit 1
