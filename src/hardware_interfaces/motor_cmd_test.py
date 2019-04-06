import rospy
from geometry_msgs.msg import Vector3 

def main():
	rospy.init_node('motor_cmd_test_node')
	publisher = rospy.Publisher('/pacmouse/motors/cmd', Vector3, queue_size=1)
	cmd = Vector3()

	while not rospy.is_shutdown():
		val = float(input('Enter a motor command:\t'))
		cmd.x = val
		cmd.y = val
		publisher.publish(cmd)


if __name__ == '__main__':
	main()