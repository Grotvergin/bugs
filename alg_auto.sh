#!/bin/bash
clear
cur_shell=$(basename $SHELL)
cur_shell=$(echo $cur_shell | sed 's/-//g')
source /opt/ros/noetic/setup.$cur_shell
catkin_make
rm -rf devel/lib/bug_alg
cp -r build/bug_alg devel/lib
source devel/setup.$cur_shell
roslaunch bug_alg visbug21.launch des_x:=2 des_y:=1 initial_x:=-1 initial_y:=0
