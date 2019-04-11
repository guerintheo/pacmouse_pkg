import rospy
import numpy as np
from pacmouse_pkg.msg import Drive
from geometry_msgs.msg import Vector3

from pacmouse_pkg.src.estimation_control.dynamics import motion_model, inverse_motion_model
from pacmouse_pkg.src.estimation_control.control import step, get_sp
from pacmouse_pkg.src.utils.maze import Maze2
import time

MAX_RATE = 10

class GoTo:
	def __init__(self):

		self.origin = None
		self.target = None
		self.prev_time = time.time()


		self.maze = Maze2()
		self.maze.load("../utils/mini.maze")
		self.mode = 'coord'


		rospy.init_node('motor_encoder_tester')
		rospy.Subscriber('/pacmouse/pose/mocap', Vector3, self.pose_callback)
		self.cmd_pub = rospy.Publisher('/pacmouse/motor/cmd', Drive, queue_size=1)
		self.spin()


	def pose_callback(self, msg):
		if time.time() - self.prev_time < 1./MAX_RATE:
			return
		self.prev_time = time.time()

		self.pose = np.array([msg.x, msg.y, msg.z])

		if self.origin is None:
			self.origin = self.pose[:2]

		if self.mode == 'coord':

			cmd = Drive()
			cmd.L = 0
			cmd.R = 0

			if self.target is not None:
				cmd.L, cmd.R = inverse_motion_model(step(self.pose, self.target))

			self.cmd_pub.publish(cmd)
		elif self.mode == 'cell':
			cmd = Drive()
			cmd.L = 0
			cmd.R = 0

			if self.target is not None:
				set_point = get_sp(self.pose, self.maze, self.target.astype(int))
				cmd.L, cmd.R = inverse_motion_model(step(self.pose, self.target))

			self.cmd_pub.publish(cmd)

		elif self.mode == 'speed':
			cmd = Drive()
			cmd.L = 0
			cmd.R = 0

			if self.target is not None:
				cmd.L = self.target[0] - self.origin[0]
				cmd.R = self.target[1] - self.origin[1]
			self.cmd_pub.publish(cmd)


	def spin(self):

		while not rospy.is_shutdown():
			if self.origin is None:
				print 'Waiting for mocap pose data'
				time.sleep(1)
			else:
				raw = raw_input('Enter a position x y:\t').strip()
				try:
					val_x, val_y = raw.split(' ')
					x = float(val_x)
					y = float(val_y)
					self.target = np.array([x,y]) + self.origin
				except:
					self.target = None
					if raw.lower() == 'zero':
						self.origin = None
					elif raw.lower() == 'cell':
						self.mode = 'cell'
						print 'Mode set to cell'
					elif raw.lower() == 'coord':
						self.mode = 'coord'
						print 'Mode set to coord'
					elif raw.lower() == 'speed':
						self.mode = 'speed'
						print 'Mode set to speed'
					else:
						print 'Failed to parse "{}". Setting velocity to zero.'.format(raw)
					


if __name__ == '__main__':
	gt = GoTo()
