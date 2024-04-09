#!/usr/bin/env python2

# Common packages
import numpy as np

# Major ROS
import rospy

# Orocos Kinematic Dynamic Library (KDL)
import PyKDL
import kdl_parser_py.urdf as urdf

# Services
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

# Messages
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState



class iiwa:
	# Part of the class I get from https://github.com/wuphilipp/sawyer_kdl/blob/master/scripts/sawyer_jacobian.py
	# Better description was found in https://github.com/RethinkRobotics/baxter_pykdl/blob/master/src/baxter_pykdl/baxter_pykdl.py
	def __init__(self):
		self.base_link = 'iiwa_link_0'
		self.end_effector = 'tool_link_ee_kuka'
		flag, self.tree = urdf.treeFromParam('/robot_description')
		self.arm_chain = self.tree.getChain(self.base_link, self.end_effector)

		# Gravity vector
		self.gravity = PyKDL.Vector(0, 0, -9.80) # From gazebo world

		# The names of joints
		self.joint_names = np.array(['iiwa_joint_{}'.format(i) for i in range(1, 8)])

		# Number of joints (7)
		self.n = self.arm_chain.getNrOfJoints()

		# Joint positions and velocities
		self.joint_positions_kdl = PyKDL.JntArray(self.n)
		self.joint_positions = np.zeros([self.n, 1])
		self.joint_velocities_kdl = PyKDL.JntArray(self.n)
		self.joint_velocities = np.zeros([self.n, 1])

		# Kinematics and dynamics of the robot
		# Methods allow to get the Jacobian matrix
		# and Dynamic model matrices as M (Inertia matrix), C (Corioliss matrix) and G (Gravity matrix)
		self.jacobian_kdl = PyKDL.ChainJntToJacSolver(self.arm_chain)
		self.dynamics_kdl = PyKDL.ChainDynParam(self.arm_chain, self.gravity)

		# Dynamic model matrices
		self.G = PyKDL.JntArray(self.n)
		self.Cdq = PyKDL.JntArray(self.n)
		self.M_kdl = PyKDL.JntSpaceInertiaMatrix(self.n)
		self.M = np.mat(np.zeros((self.n, self.n)))


		# Publishers and Subscribers
		# TODO: add control to the joint states
		self.joint_states_sub = rospy.Subscriber("/iiwa/joint_states", JointState, self.update_joints_state)
		self.torque_control_pub = [rospy.Publisher('/iiwa/joint{}_torque_controller/command'.format(i), Float64, queue_size = 10) for i in range(1, self.n+1)]

		# Test angles
		self.pos_test = np.zeros([self.n, 1])
		self.pos_test[0] = 0
		self.pos_test[1] = 0.5
		self.pos_test[2] = 0
		self.pos_test[3] = -0.5
		self.pos_test[4] = 0
		self.pos_test[5] = 2
		self.pos_test[6] = 0

		self.err = self.pos_test

		# Control torque
		self.tau = np.ones([self.n, 1])
		self.Kp = np.identity(self.n)*20
		self.Kd = np.identity(self.n)*1

		# self.torque_control_pub[0].publish(0.0)
		# tmp = self.Kp.dot(self.joint_positions)

	def update_joints_state(self, joint_state_msg):

		for i in range(self.n):
			self.joint_positions_kdl[i] = joint_state_msg.position[i]
			self.joint_positions[i] = joint_state_msg.position[i]
			self.joint_velocities_kdl[i] = joint_state_msg.velocity[i]
			self.joint_velocities[i] = joint_state_msg.velocity[i]
		
		# Control
		self.dynamics_kdl.JntToGravity(self.joint_positions_kdl, self.G)
		self.dynamics_kdl.JntToCoriolis(self.joint_positions_kdl, self.joint_velocities_kdl, self.Cdq)
		self.dynamics_kdl.JntToMass(self.joint_positions_kdl, self.M_kdl)
		self.kdl_to_mat(self.M_kdl, self.M)
		self.tau = (self.Kp.dot(self.pos_test - self.joint_positions))

		for i in range(self.n):
			self.torque_control_pub[i].publish(self.tau[i] - self.Kd[i, i]*self.joint_velocities_kdl[i] + self.Cdq[i] + self.G[i])


	def kdl_to_mat(self, mat_in, mat_out):
		for i in range(mat_in.rows()):
			for j in range(mat_in.columns()):
				mat_out[i,j] = mat_in[i,j]




def setupControlManager():
	# Turn on torque control
	rospy.loginfo('Turn on torque controllers')

	# Create Client object
	switchTorqueController = rospy.ServiceProxy('/iiwa/controller_manager/switch_controller', SwitchController)

	# Fill the fields
	start_controllers = np.array(['joint{}_torque_controller'.format(i) for i in range(1, 8)])
	stop_controllers = np.array([])
	strictness = SwitchControllerRequest.BEST_EFFORT
	start_asap = False
	timeout = 0 # Infinite

	# Send service message
	resp = switchTorqueController(start_controllers, stop_controllers, strictness, start_asap, timeout)

	# Print out the response information
	rospy.loginfo(resp)


if __name__ == '__main__':
	rospy.init_node('iiwa_force_control')
	
	setupControlManager()
	rospy.sleep(1.0)

	iiwa_robot = iiwa()

	rospy.spin()
