#!/usr/bin/python
import rospy
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

from pacmouse_pkg.msg import Drive
from pacmouse_pkg.src.estimation_control.control import step
from pacmouse_pkg.src.estimation_control.dynamics import inverse_motion_model


class ControllerNode:
	def __init__(self):
		rospy.init_node('controller_node')

		self.pose = np.zeros(3)
		self.plan = np.zeros(3)
		self.arm = False

		self.cmd_pub = rospy.Publisher('/pacmouse/motor/cmd', Drive, queue_size=1)
		rospy.on_shutdown(self.shutdown)
		rospy.Subscriber('/pacmouse/pose/mocap', Vector3, self.pose_callback)
		rospy.Subscriber('/pacmouse/pose/estimate', Vector3, self.pose_callback)
		rospy.Subscriber('/pacmouse/plan', Vector3, self.plan_callback)
		rospy.Subscriber('/pacmouse/mode/set_motor_arm', Bool, self.arm_callback)

		rospy.spin()

	def pose_callback(self, msg):
		self.pose[0] = msg.x
		self.pose[1] = msg.y
		self.pose[2] = msg.z

		cmd = Drive()
		cmd.L = 0
		cmd.R = 0

		if self.arm and\
		   np.linalg.norm(self.pose[:2] - self.plan[:2]) > 0.005:

			forward, turn = step(self.pose, self.plan)
			print 'Drive forward at {} meters/second.\tTurn at {} radians/second.'.format(forward, turn)
 			cmd.L, cmd.R = inverse_motion_model((forward,turn))

		self.cmd_pub.publish(cmd)

	def plan_callback(self, msg):
		self.plan[0] = msg.x
		self.plan[1] = msg.y
		self.plan[2] = msg.z
		print 'Received plan {} {} {}'.format(*self.plan)

	def arm_callback(self, msg):
		self.arm = msg.data
		print 'Armed' if self.arm else 'Disarmed'

	def shutdown(self):
		cmd = Drive()
		cmd.L = 0
		cmd.R = 0
		self.cmd_pub.publish(cmd)
		print 'Received shutdown.'


if __name__ == '__main__':
	controller = ControllerNode()
