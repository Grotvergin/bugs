#!/usr/bin/env python
import random
import math
import sys
import os

def main(mapcase,mname, fname):
    #create file in turtlebot3.launch
    os.system("cd ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch &&touch " + mname+"_world.launch")
    output_stream = os.popen("cd ~/ && pwd")
    main_dir = output_stream.read()
    #Create file and write new data
    f  = open(main_dir[:-1]+"/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/"+ mname+"_world.launch", "w")
    #different spawn points for different map types
    if mapcase == '0':
        f.write("""<launch>
          <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
          <arg name="x_pos" default="1.5"/>
          <arg name="y_pos" default="1.5"/>
          <arg name="z_pos" default="0.5"/>

          <include file="$(find gazebo_ros)/launch/empty_world.launch">""")
    else:
        f.write("""<launch>
          <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
          <arg name="x_pos" default="2.5"/>
          <arg name="y_pos" default="2.5"/>
          <arg name="z_pos" default="0.5"/>

          <include file="$(find gazebo_ros)/launch/empty_world.launch">""")

    f.write(f'\n        <arg name="world_name" value="'+fname +'/'+mname+'_world.world'+'"/>\n')
    f.write("""        <arg name="paused" value="false"/>
        <arg name="use_sim_time" value="true"/>
        <arg name="gui" value="true"/>
        <arg name="headless" value="false"/>
        <arg name="debug" value="false"/>
      </include>

      <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

      <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
    </launch>""")
    f.close()
if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
