# rt2_assignment1 - branch 'ros2' - Matteo Azzini 4475165

## Requirements
In this branch I was required to write FSM and position server nodes for ROS2, as components, so that, by using the *ros1_bridge*, they can be interfaces with the ROS nodes and with the simulation in Gazebo. The *go_to_point* can still be implemented as a service.

## Package Composition
The package contains the folllowing elements:</br>
- **doc**: folder with documentation about the package, generated with doxygen </br>
- **include**: folder required for a ROS2 package </br>
- **launch**: folder with a launch file to start a container and load the two components written in cpp </br>
- **src**: folder with the nodes written in cpp as components</br>
- **srv**: folder with .srv files to define the services</br>
- **CMakeList.txt**: file required to build the pacakge</br>
- **mapping_rules.yaml**: file in .yaml extension to define which packages and messages/services should be bridged for the communication ROS-ROS2</br>
- **pacakage.xml**: file required to build the package</br>
- **start_sim_ros2_gazebo.sh**: script to start the whole simulation on Gazebo</br>
- **start_sim_ros2_coppelia.sh**: script to start the whole simulation on Vrep</br>
- **scene.ttt**: Vrep scene 

## Code Explanation

### Launch files
- **launch.py**: file to start a container and load inside two nodes written as components in src folder

### src
- **state_machine.cpp**: implements a ROS2 component which declares a server for Command server , a client for RandomPosition service and a client for Position service. It receives the request from *user_interface* node in ROS, if user requires to start the robot, this component sends a request for a random position and call *go_to_point* node to let the robot reach the goal. Otherwise, if the robot is moving and user requests to stop it, the component doesn't do anything, because *go_to_point* node stop automatically the robot when the goal is reached 

- **position_server.cpp**: implements a server, as ROS2 component, which receives as request the minimum and maximum value, within choose x and y coordinates. As response returns random x, y coordinates and the heading theta for a goal position

### srv
- **Command.srv**: Definition of Command service used to send a command to control the robot  </br>
- **Position.srv**: Definition of Position service used to reach a goal position</br>
- **RandomPosition.srv**: Definition of RandomPosition used to get a random goal position</br>

## Required Packages
To test this code is required the *ros1_bridge* package and *rt2_assignment1* package in its original version, available in this repository, in branch 'main'.

## How to execute the code
### Simulation on Gazebo manually
1. Download and build this package in the src folder of your ROS2 worksapace, the same should be done for *ros1_bridge* package
2. Download and build *rt2_assignment1* package in its original version in the src folder of your ROS workspace
3. Open the first terminal, source ROS environment, go into your ROS workspace and execute 
 
```
roslaunch rt2_assignment1 sim_ros2_gazebo.launch
```

N.B. *sim_ros2_gazebo.launch* file is available in launch folder of branch 'main' of rt2_assignment1 repository </br>

4. Open a second terminal, source ROS and ROS2 environments, go into your ROS2 workspace and execute  

```
ros2 run ros1_bridge dynamic_bridge
```

5. Open a third terminal, source ROS2 environment, go into your ROS2 workspace and execute

```
ros2 launch rt2_assignment1 launch.py
```

N.B. sometimes may happen that in the third terminal an error occours referring to an unknown synmbol, in this case you can simply re-execute the code but sourcing also ROS in the third terminal

### Simulation on Gazebo by a script
Points 3,4 and 5 can be replaced by a script already availabe in this package. Copy this script in your root folder, make it executable and the execute it with:

```
./start_sim_ros2_gazebo.sh
```

N.B. Before use this script you have to install *gnome-terminal* to open multiple terminals by script. Pay also attention to the path for workingspaces in the script, you have to modify them in according with your workingspaces paths starting from the folder within you downloaded the script

### Simulation on Vrep manually
To execute this simulation on Vrep instead of Gazebo follow the procedure above replacing *sim_ros2_gazebo.launch* with *sim_ros2_coppelia.launch* in point 3 ( file available in 'main' branch of rt2_assignment1 repository)
Once you have completed all previous steps, to start Vrep simulator you have to open a new terminal, source ROS environment, go into *CoppeliaSim_Edu_V4_2_0_Ubuntu20_04* folder and execute:

```
./coppeliaSim.sh
```

From Vrep GUI you can open the scene *scene.ttt* available in this branch or in 'main' branch and start the simulator with the play button.
N.B. If you dont't have already *CoppeliaSim_Edu_V4_2_0_Ubuntu20_04* folder, you can download it from this link:
http://www.coppeliarobotics.com/downloads.html

### Simulation on Vrep by a script
As before you can run the simulation using the script *start_sim_ros2_coppelia.sh* available in this package:

```
./start_sim_ros2_coppelia.sh
```

N.B. Before use this script you have to install *gnome-terminal* to open multiple terminals by script. Pay also attention to the path for workingspaces in the script, you have to modify them in according with your workingspaces paths starting from the folder within you downloaded the script. Also  the path of *CoppeliaSim_Edu_V4_2_0_Ubuntu20_04* folder should be modified 
