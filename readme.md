# Home Service Robot Project

Home service robot is the capstone project of the [Robotics Specialization Course On Udacity](https://www.udacity.com/enrollment/nd209).

The project's aim is to practice the skills I gained in the course. Listed below are the concepts I practiced:

1. SLAM to map the environment.
2. Localization for the robot to localize itself in the environment
3. Navigation based on Dijkstra's algorithm.

The packages I used to achieve the above are:
1. SLAM_Gmapping for SLAM
2. AMCL (Adaptive Monte Carlo Localization) for localization in the mapped environment
3. ROS navigation stack was used for navigation to simulate picking and dropping off objects at specified locations, it uses Dijkstra's algorithm under the hood.

This project was done using ROS Kinetic which runs on Ubuntu 16.04LTS

To run the project, Follow the instructions below:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
sudo apt-get update
cd ~/catkin_ws/src
git clone https://github.com/usmahm/home-server-bot.git .

git clone https://github.com/ros-perception/slam_gmapping
git clone https://github.com/turtlebot/turtlebot
git clone https://github.com/turtlebot/turtlebot_interactions
git clone https://github.com/turtlebot/turtlebot_simulator
cd ~/catkin_ws/
source devel/setup.bash
rosdep -i install gmapping
rosdep -i install turtlebot_teleop
rosdep -i install turtlebot_rviz_launchers
rosdep -i install turtlebot_gazebo
catkin_make
source devel/setup.bash
```

then run 
```
cd scripts
chmod +x home_service.sh
•/home_service.sh
```

Below is a video of the running project


https://github.com/user-attachments/assets/d8d2adee-c4bb-4280-aa09-471d78e0f2d6

