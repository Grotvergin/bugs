#!/bin/bash
clear
cur_shell=$(basename $SHELL)
cur_shell=$(echo $cur_shell | sed 's/-//g')
source devel/setup.$cur_shell
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
