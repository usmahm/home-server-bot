#!/bin/sh

WORKSPACE_PATH=../../devel/setup.bash

if [! -f "$WORKSPACE_PATH" ]; then
  echo "Error: could not find $WORKSPACE_PATH. Make sure you are in the correct directory"
  exit 1
fi

xterm -e " source $WORKSPACE_PATH; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10
xterm -e " source $WORKSPACE_PATH; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 10
xterm -e " source $WORKSPACE_PATH; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm -e " source $WORKSPACE_PATH; roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 10