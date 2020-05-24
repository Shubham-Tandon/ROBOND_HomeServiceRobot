#!/bin/sh

xterm -e " roslaunch turtlebot_gazebo world.launch" &
sleep 5
export TURTLEBOT_GAZEBO_MAP_FILE='$(find turtlebot_gazebo)/maps/map.yaml'
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " rosrun add_markers add_markers"
