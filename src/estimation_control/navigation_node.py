import rospy

class NavigationNode:
	def __init__(self):
		rospy.init_node('navigation_node')


		self.initial_state = np.array([p.maze_cell_size/2, p.maze_cell_size/2, 0., 0, 0, 0])
		self.estimator = Estimator(self.initial_state, p.num_particles)

		self.encoders = np.zeros(2)
		self.imu = 0.0

		self.prev_t = time.time()

		self.maze = Maze2()


		self.prev_plan = self.initial_state[:2]


		# publishers
		self.pose_pub = rospy.Publisher('/pacmouse/pose/estimate', Vector3, queue_size=1)
		self.plan_pub = rospy.Publisher('/pacmouse/plan', Vector3, queue_size=1)

		# sensor callbacks for state estimation
		rospy.Subscriber('/pacmouse/lidars', Lidars, self.lidars_callback)
		rospy.Subscriber('/pacmouse/imu', Float64, self.imu_callback)
		rospy.Subscriber('/pacmouse/encoders/velocity', Drive, self.encoders_callback)

		# mode controller callbacks
		rospy.Subscriber('/pacmouse/mode/zero_pose', Empty, self.zero_pose_callback)
		rospy.Subscriber('/pacmouse/mode/set_plan_mode', String, self.mode_callback)

	###########################################################################
	# Estimation stuff
	###########################################################################

	def lidars_callback(self, msg):
		# the lidars message is a fixed array of size 6. we only use the
		# first five spots because we only have five working lidars :(
		# we need to convert from millimeters to meters
		self.lidars = np.array(msg.dists[:p.num_lidars]) / 1000.0

		# pose estimate updates are triggered on the lidars. yay
		self.update_pose_estimate()
		self.update_maze_estimate()

		# these functions publish a pose and a plan respectively
		self.publish_pose_estimate()
		self.replan()

	def imu_callback(self, msg):
		self.imu = msg.data

	def encoders_callback(self, msg):
		self.encoders = np.array([msg.L, msg.R])

	def zero_pose_callback(self, msg):
		print 'Zero estimated pose'
		self.estimator.set_state(self.initial_state)

	def publish_pose_estimate(self):
		msg = Vector3()
		msg.x, msg.y, msg.z = self.estimator.state[:3]
		self.pose_pub.publish(msg)

	def update_pose_estimate(self):
		t = time.time()
		dt = t - self.prev_t
		self.prev_t = t

		Z = (self.lidars, self.encoders, self.imu)
		self.estimator.update(Z, dt)


	###########################################################################
	# Planning stuff
	###########################################################################

	def replan(self):
		# if we are within a certain radius of the previous setpoint, then replan
		if np.linalg.norm(self.pose[:2] - self.prev_plan) < p.distance_to_cell_center_for_replan:
			current_index = self.maze.pose_to_index(self.pose)
			goal_index = self.goal_index[0] + self.goal_cell[1] * self.maze_width

			if self.shortest_path_solving:
				plan = self.maze.get_path(current_index, goal_index)
				if len(plan) < 2:
					print 'The plan is too short... I think we made it??!?'
					target_index = current_index
				else:
					target_index = plan[1]

			else:
				target_index = self.tremaux.get_plan(current_index, self.maze)
				print 'Tremaux {}'.format(target_index)
				if self.tremaux.min_count > 0:
					print 'We\'ve explored the whole maze!'

			target_coord = self.maze.index_to_cell_center(target_cell)
			self.prev_plan = target_coord
			msg = Vector3()
			msg.x = target_coord[0]
			msg.y = target_coord[1]
			self.plan_publisher.publish(msg)

	def mode_callback(self, msg):
		if msg.data == 'EXPLORING':
			self.shortest_path_solving = False
		elif msg.data == 'SHORTEST_PATH_SOLVING':
			if self.goal_cell is None:
				print 'We haven\'t found the goal cell yet. No shortest path solve'
			else:
				self.shortest_path_solving = True
		else:
			print 'Mode {} not recognized.'.format(msg.data)


if __name__ == '__main__':
	ros_is_NOT_a_nice_guy = EstimationNode()