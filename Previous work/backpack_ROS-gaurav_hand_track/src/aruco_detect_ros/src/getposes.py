#!/usr/bin/env python 
import numpy as np
import hebi
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import rospy
import make_kin_utils
import varsList as vL

# from http://docs.hebi.us/tools.html#python-api
class POSE(object):
	"""Class to wrap subscribing the pose values and the joint state values"""
	def __init__(self):
		super(POSE, self).__init__()
		#Random initialization 
		self.x = -215
		self.y = -521
		self.z = -512 
		self.fb_position = [0, 0, 0]

	# Callback for pose from aruco
	def __callback__(self,data):
		self.x=data.pose.position.x
		self.y=data.pose.position.y
		self.z=data.pose.position.z
		# self.UPDATE=True

	# Joint state callback
	def fb_pose(self, data):
		self.fb_position = data.position

	# Subscriber node
	def getxyz(self):
		self.UPDATE=False	
		rospy.Subscriber("/aruco_single/pose", PoseStamped, self.__callback__)
		self.fb = rospy.Subscriber("/hebiros/right_arm/feedback/joint_state", JointState, self.fb_pose)

	# Print values of aruco marker
	def printval(self):
		print("X:{}\tY:{}\tZ:{}".format(self.x,self.y,self.z))


if __name__ == '__main__':
	#Initialize node
	rospy.init_node('detect_marker', anonymous=True)
	# Declaring object of POSE class
	pose = POSE()
	
	# Initialize backpack joint object
	kin, joint_mask = make_kin_utils.make_backpack_kin() 
	group = hebi.Lookup().get_group_from_names(['*'], ['base_temp', 'X-00476', 'elbow_temp']) #Group the joints
	
	# Add offset for transformation from camera to robot model
	offset = np.array([0.0,-0.4,0.0])

	alpha=0.25 #Particle filter alpha value
	PI=3.142  #constant
	xyz = np.array([.4,0,1]) #initialize xyz
	i=0 #iterator
	rate = rospy.Rate(5.0)
	while not rospy.is_shutdown():
		
		# Get value from subscribing from the topic of aruco.
		pose.getxyz()
		
		# print(xyz)
		# Apply offset
		xyznew=np.array([pose.x+offset[0],pose.y+offset[1], pose.z+offset[2]])
		xyz=alpha*xyz+(1-alpha)*xyznew #Particle filter


		print("Detected position: ",xyznew)
		print("Goal Position: ", xyz)

		#Changing orientation of the axis from camera model to robot model
		xyz_hebi = xyz
		xyz_hebi[0] = xyz[2]
		xyz_hebi[1] = xyz[0]
		xyz_hebi[2] = xyz[1]

		# get end effector objective from hebi given a position goal
		end_effector_position_objective = hebi.robot_model.endeffector_position_objective(xyz_hebi, weight=1.0)
		
		# SOlve ikev
		positions = kin.solve_inverse_kinematics(pose.fb_position, end_effector_position_objective)#Check the output of fb_position
		
		# Making the range of ikev between -pi/2 to pi/2
		positions=positions%(2*PI)
		if(positions[0]>3*PI/2):
			positions[0]=positions[0]-2*PI
		if(positions[0]>PI/2):
			positions[0]=PI/2

		if(positions[1]>3*PI/2):
			positions[1]=positions[1]-2*PI
		if(positions[1]>PI/2):
			positions[1]=PI/2

		if(positions[2]>3*PI/2):
			positions[2]=positions[2]-2*PI
		if(positions[2]>PI/2):
			positions[2]=PI/2

		# Negative axis
		positions[0] = -positions[0]
		positions[1] = -positions[1]
		print("Joint angles:", positions)
                
		# Send the command for joint angles
		group_command = hebi.GroupCommand(3)
		group_command.position = positions
		group.send_command(group_command)
       
		# Iterate
		i=i+1
		rate.sleep()

