import rospy
import pacmouse_pkg.src.params as p
from apa102_led import apa102
import time

OFF = 0x000000 #Off
RED = 0xFF0000 #Red
ORANGE = 0xFF7F00 #Orange
YELLOW = 0xFFFF00 #Yellow
GREEN = 0x00FF00 #Green
BLUE = 0x0000FF #Blue
INDIGO = 0x4B0082 #Indigo
VIOLET = 0x9400D3 #Violet

class LEDs: 
	def __init__(self):
		rospy.init_node('led_status_node')
		# master_status_sub = rospy.Subscriber('/master_status', MasterStatusMsg, master_status_callback)
		self.strip = apa102.APA102(num_led=p.num_leds, global_brightness=p.led_default_brightness, mosi=p.mosi, sclk=p.sclk, order='rgb')
		self.rainbow()
		rospy.on_shutdown(self.shutdown) # TODO: Ask ros geniuses if this is the right way to shut down.

	def shutdown(self):  
		self.strip.clear_strip()
		self.strip.cleanup()

	def setall(self, color):
		for i in range(p.num_leds):
			self.strip.set_pixel_rgb(i,color)	
		self.strip.show()

	def rainbow(self):
		self.strip.set_pixel_rgb(0,RED)
		self.strip.show()
		time.sleep(0.2)
		self.strip.set_pixel_rgb(0,ORANGE)
		self.strip.show()
		time.sleep(0.2)
		self.strip.set_pixel_rgb(0,YELLOW)
		self.strip.show()
		time.sleep(0.2)
		self.strip.set_pixel_rgb(0,GREEN)
		self.strip.show()
		time.sleep(0.2)
		self.strip.set_pixel_rgb(0,BLUE)
		self.strip.show()
		time.sleep(0.2)
		self.strip.set_pixel_rgb(0,PURPLE)
		self.strip.show()
		time.sleep(0.2)
		self.strip.set_pixel_rgb(0,OFF)
		self.strip.show()
		time.sleep(0.2)

	def master_status_callback(self, msg):
		if msg == "ok":
			self.strip.set_pixel_rgb(0,GREEN)

		self.strip.show()

				


if __name__ == '__main__':
	l = LEDs()