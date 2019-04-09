import rospy
import numpy as np
from std_msgs.msg import Bool

class ButtonTester:
	def __init__(self):
		rospy.init_node('button_tester')
		self.publishers = dict()
		for i, pin in enumerate(p.button_pins):
			self.publishers[i] = rospy.Publisher('/pacmouse/buttons/{}'.format(i+1), Bool, queue_size=1)

		self.spin()

	def spin(self):

		print 'Enter a button and its value. \ne.g.'1 0' sets button 1 to off \n'3 1' sets button 3 to on'
		while not rospy.is_shutdown():
			raw = raw_input('Enter a button and its value:\t').strip()
			try:
				button, state = raw.split(" ")
				button = int(button)
				state = bool(state)
			except:
				print 'Failed to parse "{}"'.format(raw)
			print("Publishing button {} to {}".format(button, state))
			self.publishers[button].publish(state)


if __name__ == '__main__':
	but = ButtonTester()
