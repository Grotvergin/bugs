#!/bin/bash
cur_shell=$(basename $SHELL)
cur_shell=$(echo $cur_shell | sed 's/-//g')
source /opt/ros/noetic/setup.$cur_shell
catkin_make
rm -rf devel/lib/bug_alg
cp -r build/bug_alg devel/lib
source devel/setup.$cur_shell
roslaunch bug_alg class1.launch des_x:=10 des_y:=-10
