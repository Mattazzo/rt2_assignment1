# rt2_assignment1 - branch 'ros2'

## Requirements
In this branch I was required to write FSM and position server nodes for ROS2, as components, so that, by using the *ros1_bridge*, they can be interfaces with the ROS nodes and with the simulation in Gazebo. The *go_to_point* can still be implemented as a service.

## Package Composition
The package contains the folllowing elements:</br>
	-**doc**: folder with documentation about the package, generated with doxygen </br>
	-**include**: folder required for a ROS2 package </br>
	-**launch**: folder with a launch file to start a container and load the two components written in cpp </br>
	-**src**: folder with the nodes written in cpp as components</br>
	-**srv**: folder with .srv files to define the services</br>
	-**CMakeList.txt**: file required to build the pacakge</br>
	-**mapping_rules.yaml**: file in .yaml extension to define which packages and messages/services should be bridged for the communication ROS-ROS2</br>
	-**pacakage.xml**: file required to build the package</br>
	-**start_sim_ros2_gazebo.sh**: script to start the whole simulation on Gazebo</br>
	-**start_sim_ros2_coppelia.sh**: script to start the whole simulation on Vrep</br>

## Code Explanation

### Launch files</br>
-**launch.py**: file to start a container and load inside two nodes written as components in src folder

### src</br>
-**state_machine.cpp**: implements a ROS2 component that declares a server for Command server , a client for RandomPosition service and a client for Position service. It receives the request from *user_interface* node in ROS, if user require to start the robot, this component sends a request for a random position and call *go_to_point* node to let the robot reach the goal. Otherwise, if the robot is moving and user requests to stop it, the component doesn't do anything, because *go_to_point* node stop automatically the robot when the goal is reached 

-**position_server.cpp**: implements a server, as ROS2 component, which receives as request the minimum and maximum value, within choose x and y coordinates. As response returns random x, y coordinates and the heading theta for a goal position

### srv</br>
-**Command.srv**: Definition of Command service used to send a command to control the robot  </br>
-**Position.srv**: Definition of Position service used to reach a goal position</br>
-**RandomPosition.srv**: Definition of RandomPosition used to get a random goal position</br>

## Required Packages
To test this code is required the *ros1_bridge* package and *rt2_assignment1* package in its original version, available in this repository, in branch 'main'.

## How to execute the code
### Simulation on Gazebo manually
1. Download and build this package in the src folder of your ROS2 worksapace, the same should be done for *ros1_bridge* package
2. Download and build *rt2_assignment1* package in the src folder of your ROS workspace
3. Open the first terminal, source ROS environment, go into your ROS workspace and execute 
```
roslaunch rt2_assignment1 sim_ros2_gazebo.launch
```
N.B. *sim_ros2_gazebo.launch* file is available in launch folder of branch 'main' of rt2_assignment1 repository
4. Open a second terminal, source ROS and ROS2 environments, go into your ROS2 workspace and execute  
```
ros2 run ros1_bridge dynamic_bridge
```
5. Open a third terminal, source ROS2 environment, go into your ROS2 workspace and execute
```
ros2 launch rt2_assignment1 launch.py
```

### Simulation on Gazebo by a script
Points 3,4 and 5 can be replaced by a script already availabe in this package. Copy this script in your root folder, make it executable and the execute it with:
```
./start_sim_ros2_gazebo.sh
```
N.B. Before use this script you have to install *gnome-terminal* to open multiple terminals by script by running on your terminal. Pay also attention to the path for workingspaces in the script, you have to modify them in according with your workingspaces paths

### Simulation on Vrep manually
To execute this simulation on Vrep instead of Gazebo follow the procedure above replacing *sim_ros2_gazebo.launch* with *sim_ros2_coppelia.launch* in point 3 ( file available in 'main' branch of rt2_assignment1 repository)
Once you have completed all previous steps, to start Vrep simulator you have to open a new terminal, source ROS environment, go into *CoppeliaSim_Edu_V4_2_0_Ubuntu20_04* folder and execute:
```
./coppeliaSim.sh
```
From Vrep GUI you can open the scene *scene.ttt* available in 'main' branch of rt2_assignment repository and start the simulator with the play button.
N.B. If you dont't have already *CoppeliaSim_Edu_V4_2_0_Ubuntu20_04* folder, you can download it from this link:
http://www.coppeliarobotics.com/downloads.html

### Simulation on Vrep by a script
As before you can run the simulation using the script *start_sim_ros2_coppelia.sh* available in this package:
```
./start_sim_ros2_coppelia.sh
```
N.B. Before use this script you have to install *gnome-terminal* to open multiple terminals by script by running on your terminal. Pay also attention to the path for workingspaces in the script, you have to modify them in according with your workingspaces paths, also fro the path of *CoppeliaSim_Edu_V4_2_0_Ubuntu20_04* folder
