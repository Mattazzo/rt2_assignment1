/**
 * \file state_machine.cpp
 * \brief This file implement a state machine that control the robot
 * \author Matteo Azzini
 * \version 1.0
 * \date 23/07/2021
 * 
 * \details
 * 
 * Publishers to:<BR>
 * 	° /position_server
 * 	
 * Services:<BR>
 *	° /user_interface
 * 
 * Clients:<BR>
 * 	° /position_server
 * 
 * Action Client:<BR>
 * 	° /position	 	 
 * 
 * 
 * Description:
 * 
 * State_machine node which controls the robot, interact with random 
 * position service to get a random goal and with position action server 
 * to reach it. This node implemtn also a server for user_command service, 
 * to let user control the robot.
 * 
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "rt2_assignment1/PositionAction.h"


bool start = false; /*!< variable use to start the robot if it is true */

/**
 *\brief Server function to gstart and stop robot
 * 
 * \param req: server request to start and stop robot random behavior 
 * \param res: server response
 * 
 * \return always true
 *
 * Function of user interface server, it start or stop the robot depending 
 * on user request 
 */
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/**
 *\brief Main function to control robot behavior
 * 
 * \param argc: number of argument 
 * \param argv: pointer to argument vector
 * 
 * \return always zero
 * 
 * Main function with declaration of a server for user_interface service,
 * a client to get a random position and an action client to reach the 
 * desired position. If user requested to start the robot, the node make 
 * a request to get a random position and send this position as goal for 
 * the action server. Then a timer of 30 second starts, if robot reach the
 * goal before the expiring of the timer, it prints the action status SUCCEDED, 
 * if goal is canceled, it prints the action status PREEMPTED, otherwise,
 * when timer expires, it prints "Action did not finish before the time out."
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   //ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
   
   //go_to_point action client
   actionlib::SimpleActionClient<rt2_assignment1::PositionAction> client_pos("position", true);   
   client_pos.waitForServer(); //will wait for infinite time
   
   //publisher fro action status
   ros::Publisher status_pub = n.advertise<std_msgs::String>("action_status", 100);  
   std_msgs::String status;
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
   		rt2_assignment1::PositionGoal goal;
		goal.x = rp.response.x;
		goal.y = rp.response.y;
		goal.theta = rp.response.theta;
		client_pos.sendGoal(goal);
		//status PENDING
		status.data = client_pos.getState().toString().c_str();
		status_pub.publish(status);
   		//std::cout<< "status: "<< status << std::endl;
   		
   		std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
   		
   		//wait for the action to return
		bool finished_before_timeout = client_pos.waitForResult(ros::Duration(60.0)); //true if the return is done before timer expire(60 sec), otherwise false
		
		if (finished_before_timeout)
		{
			
			ROS_INFO("Action finished: %s",client_pos.getState().toString().c_str());
			//status SUCCEEDED or PREEMPTED
			status.data = client_pos.getState().toString().c_str();
			status_pub.publish(status);
			
		}
		else {
			ROS_INFO("Action did not finish before the time out.");
			//status ACTIVE
			status.data = client_pos.getState().toString().c_str();
			status_pub.publish(status);
		}
   	}
   	
   	ros::Duration(0.5).sleep();
   }
   return 0;
}
