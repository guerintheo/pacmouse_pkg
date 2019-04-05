import rospy
import numpy as np
from geometry_msgs.msg import Vector3
import time

encoder_data = np.zeros(2)
count = 0
num_speeds = 100
num_samples = 10.

def handle_encoders(msg):
	global count, encoder_data, num_speeds, num_samples
	if count < num_samples:
		encoder_data[0] += msg.x
		encoder_data[1] += msg.y
		count += 1

def main():
	global count, encoder_data, num_speeds, num_samples
	rospy.init_node('motor_encoder_correlation_node')
	rospy.Subscriber('/pacmouse/encoders', Vector3, handle_encoders)
	cmd_pub = rospy.Publisher('/pacmouse/motors/cmd', Vector3, queue_size=1)

	log_data = np.zeros([num_speeds, 3])

	cmd = Vector3()
	for i, speed in enumerate(np.linspace(0, 0.5, num_speeds)):
		print i, speed

		cmd.x = speed
		cmd.y = speed
		cmd_pub.publish(cmd)

		while count < num_samples:
			time.sleep(0.01)

		log_data[i, 0] = speed
		log_data[i, 1:] = encoder_data/num_samples
		encoder_data = np.zeros(2)
		count = 0

	np.save('motor_encoder_correlation_data.npy', log_data)

if __name__ == '__main__':
	main()