#!/bin/sh

WORKSPACE_PATH=../../devel/setup.bash
MAP_PATH=/home/usmahm/Desktop/learning/rda/nn/catkin_ws/src/map/map.yaml

if [! -f "$WORKSPACE_PATH" ]; then
  echo "Error: could not find $WORKSPACE_PATH. Make sure you are in the correct directory"
  exit 1
fi

xterm -e " source $WORKSPACE_PATH; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10
xterm -e " source $WORKSPACE_PATH; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$MAP_PATH" &
sleep 10
xterm -e " source $WORKSPACE_PATH; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm -e " source $WORKSPACE_PATH; roslaunch add_markers markers.launch" &
sleep 10
xterm -e " source $WORKSPACE_PATH; rosrun pick_objects pick_objects"