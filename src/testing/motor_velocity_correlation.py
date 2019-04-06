import rospy
import numpy as np
from geometry_msgs.msg import Vector3
import time
from pacmouse_pkg.src.hardware_interfaces.motors import Motors
from pacmouse_pkg.src.utils.math_utils import wrap

turning_rate = 0
prev_time = 0
prev_angle = 0

num_speeds = 20
num_samples = 20.
count = 0

def handle_pose(msg):
	global count, turning_rate, prev_time, prev_angle, num_speeds, num_samples
	if count < num_samples:
		count += 1
		curr_angle = msg.z
		curr_time = time.time()

		curr_rate = wrap(curr_angle - prev_angle)/(curr_time - prev_time)
		turning_rate += curr_rate

		curr_angle = prev_angle
		curr_time = prev_time

def main():
	global count, encoder_data, num_speeds, num_samples
	rospy.init_node('motor_velocity_correlation_node')
	rospy.Subscriber('/pacmouse/pose/mocap', Vector3, handle_encoders)
	m = Motors()

	log_data = np.zeros([num_speeds, 3])

	cmd = Vector3()
	for i, speed in enumerate(np.linspace(0, 0.5, num_speeds)):
		print i, speed

		m.set(-speed, speed)

		while count < num_samples:
			time.sleep(0.01)

		log_data[i, 0] = speed
		log_data[i, 1] = turning_rate/num_samples
		turning_rate = 0
		count = 0

	m.stop()

	np.save('motor_turning_correlation_data.npy', log_data)

if __name__ == '__main__':
	main()