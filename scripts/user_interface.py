"""
This script implement a ROS node as a user interface to command the robot.
User can press 1 to start the robot to let it go to a random target, 0 to 
stop the robot immediately in the current position canceling his goal.
"""

import rospy
import time
from rt2_assignment1.srv import Command
import rt2_assignment1.msg
import actionlib

def main():
	"""
	This is the main function that declare a client for the user_command 
	service and an action client to cancel the goal for the robot.
	If user press 1, a request is made for user_command server to start 
	the robot, if he press 0, the robot is stopped canceling the goal for
	the robot and making a stop request to user_command server.
	"""
	
	rospy.init_node('user_interface')
	ui_client = rospy.ServiceProxy('/user_interface', Command)
	#action client to cancel a goal
	act_client = actionlib.SimpleActionClient('position',rt2_assignment1.msg.PositionAction)
	act_client.wait_for_server()
	
	time.sleep(10)
	rate = rospy.Rate(20)
	x = int(input("\nPress 1 to start the robot "))
	while not rospy.is_shutdown():
		if (x == 1):
			ui_client("start")
			x = int(input("\nPress 0 to stop the robot "))
		elif (x == 0):
			#cancel goal
			act_client.cancel_all_goals()
			print("goal cancelled, robot is stopped")
			ui_client("stop")
			x = int(input("\nPress 1 to start the robot "))
		else:
			x = int(input("\nCommand unknown, please enter 1 to start the robot or 0 to stop the robot"))
			
            
if __name__ == '__main__':
	main()
