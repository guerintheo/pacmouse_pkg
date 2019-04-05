import pigpio
import rospy
import pacmouse_pkg.src.params as p
from std_msgs.msg import Bool
import time

class Buttons: 
	def __init__(self):
		self.pi = pigpio.pi()  # handler for Pi's GPIO pins

		self.publishers = dict()
		self.last_triggered = dict()
		self.last_state = dict()
		self.curr_state = dict()

		rospy.init_node('buttons')

		for i, pin in enumerate(p.button_pins):
			self.pi.set_mode(pin, pigpio.INPUT)
			self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
			self.pi.callback(pin, pigpio.EITHER_EDGE, self.button_callback)
			self.last_triggered[pin] = time.time()
			self.last_state[pin] = bool(self.pi.read(pin))
			self.curr_state[pin] = bool(self.pi.read(pin))
			self.publishers[pin] = rospy.Publisher('/pacmouse/buttons/{}'.format(i+1), Bool, queue_size=1)

		self.spin()

	def spin(self):
		r = rospy.Rate(p.button_node_freq)

		while not rospy.is_shutdown():
			t = time.time()

			for pin in p.button_pins:
				state_changed = self.last_state[pin] != self.curr_state[pin]
				time_elapsed = ((t - self.last_triggered[pin]) > p.button_debounce_time)
				if state_changed and time_elapsed:
					self.publishers[pin].publish(self.curr_state[pin])
					self.last_state[pin] = self.curr_state[pin]

			r.sleep()

		self.pi.stop() # cleanup pigpio resources

	def button_callback(self, pin, level, tick):
		self.curr_state[pin] = bool(level)
		self.last_triggered[pin] = time.time()


if __name__ == '__main__':
	b = Buttons()