#!/usr/bin/python
import rospy
import pacmouse_pkg.src.params as p
from pacmouse_pkg.msg import LED
from apa102_led import apa102
import time

OFF = 0x000000 #Off
RED = 0xFF0000 #Red
ORANGE = 0xFF7F00 #Orange
YELLOW = 0xFFFF00 #Yellow
GREEN = 0x00FF00 #Green
BLUE = 0x0000FF #Blue
PURPLE = 0x4B0082 #Indigo
VIOLET = 0x9400D3 #Violet

def append_hex(a, b):
    sizeof_b = 0

    # get size of b in bits
    while((b >> sizeof_b) > 0):
        sizeof_b += 1

    # align answer to nearest 4 bits (hex digit)
    sizeof_b += sizeof_b % 4

    return (a << sizeof_b) | b

class LEDs:
	def __init__(self):
		self.strip = apa102.APA102(num_led=10, global_brightness=1, mosi=p.mosi, sclk=p.sclk, order='rgb')
		self.color_wheel()
		# self.color_callback(LED(0, '0x00FF00'))
		
		rospy.init_node('led_status_node')
		# master_status_sub = rospy.Subscriber('/master_status', MasterStatusMsg, master_status_callback)

		rospy.Subscriber('/pacmouse/mode/set_leds', LED, self.color_callback)

		
		rospy.on_shutdown(self.shutdown)

		rospy.spin()

	def color_callback(self, LED):
		print 'setting {} to {}'.format(LED.led_num, int(LED.hex_color, 16))
		self.strip.set_pixel_rgb(LED.led_num,int(LED.hex_color, 16))

		self.strip.show() 

	def setall(self, color):
		for i in range(p.num_leds):
			self.strip.set_pixel_rgb(i,color)
		self.strip.show()

	def rgbtohex(self, r, g, b):
		return int(hex((r << 16) + (g << 8) + b), 16)

	def color_wheel(self):
		for i in range(255):
			color = self.rgbtohex(i, 0, 0)
			self.setall(color) 

	def rainbow(self):
		self.setall(RED)
		time.sleep(0.2)
		self.setall(ORANGE)
		time.sleep(0.2)
		self.setall(YELLOW)
		time.sleep(0.2)
		self.setall(GREEN)
		time.sleep(0.2)
		self.setall(BLUE)
		time.sleep(0.2)
		self.setall(PURPLE)
		time.sleep(0.2)

		self.strip.set_pixel_rgb(0,RED)
		self.strip.show()
		time.sleep(0.1)
		self.strip.set_pixel_rgb(1,RED)
		self.strip.show()
		time.sleep(0.1)
		self.strip.set_pixel_rgb(2,RED)
		self.strip.show()
		time.sleep(0.1)
		self.strip.set_pixel_rgb(0,GREEN)
		self.strip.show()
		time.sleep(0.1)
		self.strip.set_pixel_rgb(1,GREEN)
		self.strip.show()
		time.sleep(0.1)
		self.strip.set_pixel_rgb(2,GREEN)
		self.strip.show()
		time.sleep(0.1)
		self.strip.set_pixel_rgb(0,OFF)
		self.strip.show()
		time.sleep(0.1)
		self.strip.set_pixel_rgb(1,OFF)
		self.strip.show()
		time.sleep(0.1)
		self.strip.set_pixel_rgb(2,OFF)
		self.strip.show()
		time.sleep(0.1)

		print "rainbow test complete"

	def master_status_callback(self, msg):
		# During normal operation, we should always have at least one LED on
		# so that we can distinguish that we have not freshly rebooted.
		if msg == "ok":
			self.strip.set_pixel_rgb(0,GREEN)
		elif msg == "not ok":
			self.strip.set_pixel_rgb(0,RED)

		if msg == "Ready to start":
			self.strip.set_pixel_rgb(1,GREEN)
		elif msg == "Running Pacman AI in defensive mode":
			self.strip.set_pixel_rgb(1,YELLOW)
		elif msg == "Running Pacman AI in attach ghosts mode":
			self.strip.set_pixel_rgb(1,BLUE)
		elif msg == "Pacman AI paused by wifi command":
			self.strip.set_pixel_rgb(1,PURPLE)
		elif msg == "Pacman AI paused by button":
			self.strip.set_pixel_rgb(1,ORANGE)

		if msg == "Micromouse AI running mazefinding":
			self.strip.set_pixel_rgb(1,YELLOW)
		elif msg == "Micromouse AI running fast mode":
			self.strip.set_pixel_rgb(1,BLUE)


		if msg == "I have terminated my program":
			self.strip.set_pixel_rgb(2,GREEN)

		self.strip.show()

	def shutdown(self):
		print "received shutdown command"
		self.strip.clear_strip()
		self.strip.cleanup()




if __name__ == '__main__':
	l = LEDs()
