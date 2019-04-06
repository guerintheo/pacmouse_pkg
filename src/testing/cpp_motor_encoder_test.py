import rospy
import numpy as np
from pacmouse_pkg.msg import Drive

class MotorEncoderTester:
	def __init__(self):
		rospy.init_node('motor_encoder_tester')
		self.cmd_pub = rospy.Publisher('/pacmouse/motor/cmd', Drive, queue_size=1)
		self.spin()

	def spin(self):
		cmd= Drive()

		while not rospy.is_shutdown():
			raw = raw_input('Enter a motor speed:\t')
			try:
				val = float(raw)
				cmd.L = val
				cmd.R = val
				cmd_pub.publish(cmd)
			except:
				print 'Failed to parse', raw


if __name__ == '__main__':
	met = MotorEncoderTester()