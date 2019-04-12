import rospy
import numpy as np
import time

from pacmouse_pkg.src.estimation_control.estimation import Estimator
from pacmouse_pkg.src.utils.maze import Maze2
import pacmouse_pkg.src.params as p

from pacmouse_pkg.msg import Lidars, Drive, Maze

class EstimationNode:
	def __init__(self):
		rospy.init_node('estimation_node')

		initial_state = np.array([p.maze_cell_size/2, p.maze_cell_size/2, 0.])
		self.estimator = Estimator(initial_state, p.num_particles)

		self.encoders = np.zeros(2)
		self.yaw = 0.0

		self.prev_t = time.time()

		self.maze = Maze2()
		self.maze.load('../utils/mini.maze')
		self.estimator.set_maze(self.maze)

		self.maze_pub = rospy.Publisher('/pacmouse/maze', Maze, queue_size=1)
		self.pose_pub = rospy.Publisher('/pacmouse/pose/estimate', Vector3, queue_size=1)

		rospy.Subscriber('/pacmouse/lidars', Lidars, self.lidars_callback)
		rospy.Subscriber('/pacmouse/imu', Float64, self.imu_callback)
		rospy.Subscriber('/pacmouse/encoders/velocity', Drive, self.encoders_callback)

		rospy.spin()


	def publish_pose_estimate(self):
		msg = Vector3
		msg.x, msg.y, msg.z = self.estimator.state
		self.pose_pub.publish(msg)

	def update_pose_estimate(self):
		t = time.time()
		dt = t - self.prev_t
		self.prev_t = t

		Z = (self.lidars, self.encoders, self.imu)
		self.estimator.update(Z, dt)

	def publish_maze(self):
		msg = Maze()
		msg.width = self.maze.width
		msg.height = self.maze.height
		msg.h_walls = np.ravel(self.maze.h_walls)
		msg.v_walls = np.ravel(self.maze.v_walls)
		self.maze_pub.publish(msg)

	def lidars_callback(self, msg):
		# the lidars message is a fixed array of size 6. we only use the 
		# first five spots because we only have five working lidars :(
		self.lidars = msg.dists[:p.num_lidars]

		# pose estimate updates are triggered on the lidars. yay
		self.update_pose_estimate()
		self.publish_pose_estimate()

	def imu_callback(self, msg):
		self.yaw = msg.data

	def encoders_callback(self, msg):
		self.encoders = np.array([msg.L, msg.R])

if __name__ == '__main__':
	ros_gave_me_cancer = EstimationNode()