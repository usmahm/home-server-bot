#!/bin/sh

WORKSPACE_PATH=../../devel/setup.bash
MAP_PKG_PATH=$(rospack find map)

MAP_PATH=$MAP_PKG_PATH/maps/map.yaml
WORLD_PATH=$MAP_PKG_PATH/worlds/robotworld.world

if [! -f "$WORKSPACE_PATH" ]; then
  echo "Error: could not find $WORKSPACE_PATH. Make sure you are in the correct directory"
  exit 1
fi

xterm -e " source $WORKSPACE_PATH; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$WORLD_PATH" &
sleep 10
xterm -e " source $WORKSPACE_PATH; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$MAP_PATH" &
sleep 10
xterm -e " source $WORKSPACE_PATH; roslaunch rviz_config rviz.launch" &
sleep 10
# xterm -e " source $WORKSPACE_PATH; rosrun add_markers add_markers"

xterm -e " source $WORKSPACE_PATH; roslaunch add_markers markers.launch"
