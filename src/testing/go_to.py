from pacmouse_pkg.src.hardware_interfaces.motors import Motors
from pacmouse_pkg.src.estimation_control.control import step
from pacmouse_pkg.src.estimation_control.dynamics import inverse_motion_model
import numpy as np

import rospy
from geometry_msgs.msg import Pose2D

class GoTo:
	def __init__(self):
		self.pose = np.zeros(3)
		# self.target = np.zeros(2)
		self.target = np.array([0.25, 0.5])
		self.completion_radius = 0.025 # stop within a 2.5 cm radius

		self.m = Motors()

		rospy.init_node('go_to_node')
		rospy.Subscriber('/pacmouse/pose/mocap', Pose2D, self.handle_pose)

		self.wait_for_input()

	def handle_pose(self, msg):
		self.pose[0] = msg.x
		self.pose[1] = msg.y
		self.pose[2] = msg.theta

		if np.linalg.norm(self.pose[:2] - self.target) < self.completion_radius:
			self.m.set(0,0)
		else:
			motion_cmd = step(self.pose, self.target)
			motor_cmd = inverse_motion_model(motion_cmd)
			print self.pose, self.target, motion_cmd, motor_cmd
			self.m.set(*np.clip(motor_cmd/1000., -0.5, 0.5))

	def wait_for_input(self):
		while not rospy.is_shutdown():
			try:
				raw = raw_input('Enter a pose as "[x] [y]": ')
				r1, r2 = raw.split(' ')
				self.target = np.array([float(r1), float(r2)])
			except:
				print 'Failed to parse', raw



if __name__ == '__main__':
	gt = GoTo()
	