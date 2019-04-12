#!/usr/bin/python
import rospy
import numpy as np
import pacmouse_pkg.src.params as p
from std_msgs.msg import Bool

class ButtonTester:
	def __init__(self):
		rospy.init_node('button_tester')
		self.publishers = dict()
		for i, pin in enumerate(p.button_pins):
			self.publishers[i+1] = rospy.Publisher('/pacmouse/buttons/{}'.format(i+1), Bool, queue_size=1)

		self.spin()

	def spin(self):

		print 'Enter a button and its value. \ne.g."1 0" sets button 1 to off \n"3 1" sets button 3 to on\n'
		while not rospy.is_shutdown():
			raw = raw_input('Enter a button and its value:\t').strip()
			try:
				args = raw.split(" ")
				button = int(args[0])
				state = bool(int(args[1]))
				print("Publishing button {} to {}".format(button, state))
				self.publishers[button].publish(state)
			except:
				print 'Failed to parse "{}"'.format(raw)


if __name__ == '__main__':
	but = ButtonTester()
