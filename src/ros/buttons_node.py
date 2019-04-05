import pigpio
import rospy
import pacmouse_pkg.src.params as p
from std_msgs.msg import Bool
std_msgs Int16MultiArray

class BUTTONs: 
	def __init__(self):
		self.pi = pigpio.pi()  # handler for Pi's GPIO pins

		for pin in p.button_pins:
			self.pi.set_mode(pin, pigpio.INPUT)
			self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
			self.pi.callback(pin, pigpio.EITHER_EDGE, self.button_callback)
				
		rospy.init_node('buttons')
		button1 = rospy.Publisher('/pacmouse/buttons/1', Bool, queue_size=1)
		button2 = rospy.Publisher('/pacmouse/buttons/2', Bool, queue_size=1)
		button3 = rospy.Publisher('/pacmouse/buttons/3', Bool, queue_size=1)
		button4 = rospy.Publisher('/pacmouse/buttons/4', Bool, queue_size=1)

		self.button_publishers = [button1, button2, button3, button4]

		rospy.spin()

		# TODO: 1. Arm disarm Motors
		# TODO: 2. Resets the yaw axis
		# TODO: 3. One button enables start

	def button_callback(self, pin, level, tick):
		publisher_index = p.button_pins.index(pin)
		if level == 1 :
			self.button_publishers[publisher_index].publish(True)
		else:
			self.button_publishers[publisher_index].publish(False)


if __name__ == '__main__':
	b = BUTTONs()