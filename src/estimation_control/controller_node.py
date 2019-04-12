import rospy
import numpy as np

from std_msgs.msg import Bool

from pacmouse_pkg.msg import Drive
from pacmouse_pkg.src.estimation_control.control import step
from pacmouse_pkg.src.estimation_control.dynamics import inverse_motion_model


class ControllerNode:
	def __init__(self):
		rospy.init_node('controller_node')
		rospy.Subscriber('/pacmouse/pose/mocap', Vector3, self.pose_callback)
		rospy.Subscriber('/pacmouse/plan', Vector3, self.plan_callback)
		rospy.Subscriber('/pacmouse/mode/set_motor_arm', Bool, self.arm_callback)
		self.cmd_pub = rospy.Publisher('/pacmouse/motors/cmd', Drive, queue_size=1)

		self.pose = np.zeros(3)
		self.plan = None
		self.arm = False

		rospy.spin()

	def pose_callback(self, msg):
		self.pose[0] = msg.x
		self.pose[1] = msg.y
		self.pose[2] = msg.z

		cmd = Drive()
		cmd.L = 0
		cmd.R = 0

		if self.arm and\
		   self.plan is not None and\
		   np.linalg.norm(self.pose[:2] - self.plan[:2]) < 0.005:

			forward, turn = step(self.pose, self.target)
			print 'Drive forward at {} meters/second.\tTurn at {} radians/second.'.format(forward, turn)
 			cmd.L, cmd.R = inverse_motion_model((forward,turn))

		self.cmd_pub.publish(cmd)

	def plan_callback(self, msg):
		self.plan[0] = msg.x
		self.plan[1] = msg.y
		self.plan[2] = msg.z

	def arm_callback(self, msg):
		self.arm = msg.data

if __name__ == '__main__':
	ros_is_a_piece_of_shit = ControllerNode()