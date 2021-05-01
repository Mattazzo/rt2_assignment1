#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"


#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "rt2assignment1/PositionAction.h"

bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   //ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
   
   //my action
   actionlib::SimpleActionClient<rt2assignment1::PositionAction> client_pos("position", true);   
   client_pos.waitForServer(); //will wait for infinite time
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
   		rt2assignment1::PositionGoal goal;
		goal.x = rp.response.x;
		goal.y = rp.response.y;
		goal.theta = rp.response.theta;
		client_pos.sendGoal(goal);
   		
   		std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
   		
   		//wait for the action to return
		bool finished_before_timeout = client_pos.waitForResult(ros::Duration(30.0)); //true if the return is done befor timer expire(30 sec), otherwise false

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = client_pos.getState();
			ROS_INFO("Action finished: %s",state.toString().c_str());
		}
		else
			ROS_INFO("Action did not finish before the time out.");
 
   		std::cout << "Position reached" << std::endl;
   	}
   }
   return 0;
}
