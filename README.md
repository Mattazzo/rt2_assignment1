# rt2_assignment1 - branch raction' - Matteo Azzini 4475165

## Requirements
In this branch I was required to write *go_to_point.py* node as an ROS Action server instead of a simple server. Therefore I had also to modify the FSM node in oredr to let it be able to cancel a goal.

## Package Composition
The package contains the folllowing elements:</br>
- **action**: folder with .action file to define an action</br>
- **doc**: folder with documentation about the package, generated with doxygen </br>
- **include**: folder required for a ROS2 package </br>
- **launch**: folder with a launch files to start the simulation both on Gazebo and Vrep</br>
- **src**: folder with the nodes written in cpp </br>
- **urdf:**: folder with info about the robot to spawn it in Gazebo simulation 
- **srv**: folder with .srv files to define the services</br>
- **CMakeList.txt**: file required to build the pacakge</br>
- **pacakage.xml**: file required to build the package</br>
- **scene.ttt**: Vrep scene 

## Code Explanation

### Launch files
- **sim.launch**: file to start the simulation on Gazebo 
- **sim_coppelia.launch**: file to start the simulation on Vrep

### Scripts
- **go_to_point.py**: Action server to let the robot reach the goal position. It works as a finite state machine, when a goal position is received, it fixs robot yaw, go straight to the point and then rotate to have its heading like the desired yaw, so goal is reached. If the goal is canceled in the meanwhile robot is moving, server is preempted and so robot is stopped. 

- **user_interface.py**: ROS node to let user command the robot, it has a client for Command service and an action client for RandomPosition service. When user press 1 robot it makes a request to *state_machine.cpp* and robot start moving, otherwise, if he press 0, there is a request for COmmand server to stop the *state_machine.cpp* node, but also an action request *cancell_all_goals()* to cancel the goal for the robot and stop it as soon as possible.  

### src
- **state_machine.cpp**: ROS node with a server for Command service, a client fro RandomPosition service and an action client for Position service. When the server receives a "start" command, it sets a variable start to true and the main function has a loop that request a random goal position, then send it as a goal for the action server *go_to_point.py* in order to reach the goal. If the goal is reached before 30 seconds it prints the status SUCCEDEED, if the goal is canceled it prints the status PREEMPTED, otherwise if timer expires before reaching the goal, it prints a message to inform user that action is not finisched in time

- **position_server.cpp**: implements a ROS server which receives as request the minimum and maximum value, within choose x and y coordinates. As response returns random x, y coordinates and the heading theta for a goal position

### srv
- **Command.srv**: Definition of Command service used to send a command to control the robot  </br>
- **Position.srv**: Definition of Position service used to reach a goal position</br>
- **RandomPosition.srv**: Definition of RandomPosition used to get a random goal position</br>

## How to execute the code
### Simulation on Gazebo 
1. Download and build this package in the src folder of your ROS worksapace
2. Launch the simulation executing the following command in your ROS worksapce
```
roslaunch rt2_assignment1 sim.launch
```
N.B. Remeber to source your ROS workspace 

### Simulation on Vrep 
Repeate points 1 and 2 as before, but with a different launch file
```
roslaunch rt2_assignemnt1 sim_coppelia.launch
```
Then in another terminal you need to source again the ROS environment, than go into *CoppeliaSim_Edu_V4_2_0_Ubuntu20_04* folder ( available at this link http://www.coppeliarobotics.com/downloads.html ) and excute 
```
./coppeliaSim.sh
```
The Vrep GUI will start, from here you need to open the vrep scene *scene.ttt* available in this branch and start the simulation with the play button

