#! /usr/bin/env python

"""
This node implement a ROS Action to let the robot go to a goal position 
"""

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
#from rt2_assignment1.srv import Position
import math

import actionlib
import rt2_assignment1.msg #import PositionAction

# robot state variables
position_ = Point()	
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

class PositionAction():
	"""
	Class definition of the Action server go to point
	
	...
	
	Attributes
	----------	
	_feedback: PositionFeedback
		feedback of PositionAction
	_result: PositionResult
		result of PositionAction
		
	Methods
	----------	
	__init__(self)
		class constructor
	go_to_point(self, goal)	
		implement a state machine to let the robot reach a goal position
	"""
	
	# create messages that are used to publish feedback/result
	_feedback = rt2_assignment1.msg.PositionFeedback()
	_result = rt2_assignment1.msg.PositionResult()
	
	#constructor
	def __init__(self):
		"""Constructor"""
		self.server = actionlib.SimpleActionServer('position', rt2_assignment1.msg.PositionAction, self.go_to_point, auto_start = False)
		self.server.start()
		
	def go_to_point(self, goal):
		"""
		Method of the action server, if goal is canceled, robot is stopped.
		Otherwise, if goal is set, it works as a finite state machine with
		4 state:
			0 - Rotate robot to be headed for the goal
			1 - Go straight ahead
			2 - Robot is in the goal point, rotate to fix the heading
			3 - Goal reached, stop the robot 
			
		Parameters
		----------
			goal: goal to be reached by the robot
		"""
		#helper variable 
		success = True
		
		desired_position = Point()
		desired_position.x = goal.x
		desired_position.y = goal.y
		des_yaw = goal.theta
		change_state(0)
		
		rate = rospy.Rate(20)
			
		while not rospy.is_shutdown():
			
			#if goal canceled or when another goal is set
			if self.server.is_preempt_requested():
				rospy.loginfo('PositionAction Preempted')
				self.server.set_preempted()
				success = False
				#stop robot
				done()
				break
			
			if success:
				if state_ == 0:
					fix_yaw(desired_position)
				elif state_ == 1:
					go_straight_ahead(desired_position)
				elif state_ == 2:
					fix_final_yaw(des_yaw)
				elif state_ == 3:
					done()
					rospy.loginfo('Goal reached!')
					self._result.result = True
					self.server.set_succeeded(self._result)
					break 
					
			rate.sleep()

def clbk_odom(msg):
	"""
	Callback function of subscriber for topic /odom, get the position 
	and the quaternion of the robot
	
	Args:
		msg: message of type Odometry to get robot position and quaternion
	"""
	global position_
	global yaw_

	# position
	position_ = msg.pose.pose.position

	# yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]


def change_state(state):
	"""
	Function to change state for finite state machine in go to poin function
	
	Args:
		state: state of the finite state machine in go to point function
	"""
	global state_
	state_ = state
	print ('State changed to [%s]' % state_)
	

def normalize_angle(angle):
	"""
	Function to normalize an angle
	
	Args:
		- angle: angle to be normalized
	"""
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle

def fix_yaw(des_pos):
	"""
	Function to rotate the robot to be headed for the goal
	
	Args:
		des_pos: goal position to set the right angular velocity for rotation
	"""
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = normalize_angle(desired_yaw - yaw_)
	rospy.loginfo(err_yaw)
	twist_msg = Twist()
	if math.fabs(err_yaw) > yaw_precision_2_:
		twist_msg.angular.z = kp_a*err_yaw
		if twist_msg.angular.z > ub_a:
			twist_msg.angular.z = ub_a
		elif twist_msg.angular.z < lb_a:
			twist_msg.angular.z = lb_a
	pub_.publish(twist_msg)
	# state change conditions
	if math.fabs(err_yaw) <= yaw_precision_2_:
		#print ('Yaw error: [%s]' % err_yaw)
		change_state(1)


def go_straight_ahead(des_pos):
	"""
	Function to go straight ahead
	
	Args:
		des_pos: goal position to set the right linear velocity
	"""
	global kp_d, kp_a
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = desired_yaw - yaw_
	err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
						pow(des_pos.x - position_.x, 2))
	err_yaw = normalize_angle(desired_yaw - yaw_)
	rospy.loginfo(err_yaw)

	if err_pos > dist_precision_:
		twist_msg = Twist()
		twist_msg.linear.x = kp_d*0.3
		if twist_msg.linear.x > ub_d:
			twist_msg.linear.x = ub_d

		twist_msg.angular.z = kp_a*err_yaw
		pub_.publish(twist_msg)
	else: # state change conditions
		#print ('Position error: [%s]' % err_pos)
		change_state(2)

	# state change conditions
	if math.fabs(err_yaw) > yaw_precision_:
		#print ('Yaw error: [%s]' % err_yaw)
		change_state(0)

def fix_final_yaw(des_yaw):
	"""
	Function to rotate the robot, when he is in the right position, to 
	have the goal orientation
	
	Args:
		des_yaw: desired angle 
	"""
	err_yaw = normalize_angle(des_yaw - yaw_)
	rospy.loginfo(err_yaw)
	twist_msg = Twist()
	if math.fabs(err_yaw) > yaw_precision_2_:
		twist_msg.angular.z = kp_a*err_yaw
		if twist_msg.angular.z > ub_a:
			twist_msg.angular.z = ub_a
		elif twist_msg.angular.z < lb_a:
			twist_msg.angular.z = lb_a
	pub_.publish(twist_msg)
	# state change conditions
	if math.fabs(err_yaw) <= yaw_precision_2_:
		#print ('Yaw error: [%s]' % err_yaw)
		change_state(3)
		
def done():
	"""
	Function to stop the robot
	"""
	twist_msg = Twist()
	twist_msg.linear.x = 0
	twist_msg.angular.z = 0
	pub_.publish(twist_msg)
	
def set_velocities(msg):
	global kp_d,kp_a
	
	kp_d = msg.linear.x 
	kp_a = -msg.angular.z


def main():
	"""
	Main function with declaration of a publisher for /cmd_vel topic, a 
	subscriber for /odom topic, an action server for go to point service
	"""
	global pub_
	rospy.init_node('go_to_point')
	pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	sub_vel = rospy.Subscriber('velocities', Twist, set_velocities)
    #service = rospy.Service('/go_to_point', Position, go_to_point)
	action_server = PositionAction()
	
	rospy.spin()

if __name__ == '__main__':
	main()
