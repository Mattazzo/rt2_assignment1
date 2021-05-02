import rospy
import time
from rt2_assignment1.srv import Command
import rt2_assignment1.msg
import actionlib

def main():
	
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
		else:
			#print("Please wait, the robot is going to stop when the position will be reached")
			#cancel goal
			act_client.cancel_all_goals()
			print("goal cancelled, robot is stoppeed")
			ui_client("stop")
			x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
	main()
