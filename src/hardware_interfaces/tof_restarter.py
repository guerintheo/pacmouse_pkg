import rospy
import subprocess
import time

from pacmouse_pkg.msg import Lidars

class TofRestarter:
	def __init__(self):
		rospy.init_node('tof_restarter')
		rospy.Subscriber('/pacmouse/lidars', Lidars, self.lidars_callback)

		self.got_lidar_msg = False
		self.time_before_retry = 20. # seconds
		self.spin()

	def spin(self):
		while not rospy.is_shutdown():
			p = subprocess.Popen('rosrun pacmouse_pkg tof_pub', shell=True)

			print 'Waiting for {} seconds.'.format(self.time_before_retry)
			time.sleep(self.time_before_retry)

			if self.got_lidar_msg:
				print 'Got a lidar message! Exiting.'
				sys.exit(0)
			else:
				print 'No lidar message. Retrying.'
				self.time_before_retry += 5
				p.kill()
				time.sleep(1)

	def lidars_callback(self, msg):
		self.got_lidar_msg = True

if __name__ == '__main__':
	fuck_ros = TofRestarter()