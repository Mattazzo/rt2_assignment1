#!/bin/bash

gnome-terminal --tab --title="ros1_gazebo" -- bash -c "source ros.sh;  roslaunch rt2_assignment1 sim.launch"
gnome-terminal --tab --title="bridge" -- bash -c "sleep 3; source ros12.sh; cd Desktop/my_ros2; ros2 run ros1_bridge dynamic_bridge "
gnome-terminal --tab --title="ros2" -- bash -c "sleep 3; source ros2.sh; cd Desktop/my_ros2; ros2 launch rt2_assignment1 launch.py" 
