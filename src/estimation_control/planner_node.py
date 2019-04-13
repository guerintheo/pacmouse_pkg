#!/usr/bin/python
import rospy
import numpy as np

from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from pacmouse_pkg.msg import Maze

from pacmouse_pkg.src.estimation_control.tremaux import Tremaux
from pacmouse_pkg.src.utils.maze import Maze2
import pacmouse_pkg.src.params as p

class PlannerNode:
	def __init__(self):
		rospy.init_node('planner_node')

		self.plan_publisher = rospy.Publisher('/pacmouse/plan', Vector3, queue_size=1)

		self.pose = np.zeros(3)
		self.prev_plan = np.ones(2) * p.maze_cell_size/2

		# self.shortest_path_solving = False
		# self.maze = None

		self.shortest_path_solving = False
		self.maze = Maze2()
		self.maze.load('../utils/mini.maze')
		self.maze.build_adjacency_matrix(threshold=p.wall_transparency_threshold)
		self.maze.solve()

		self.tremaux = Tremaux(self.maze)

		self.goal_cell = np.zeros(2, dtype=int)

		rospy.Subscriber('/pacmouse/pose/mocap', Vector3, self.pose_callback)
		rospy.Subscriber('/pacmouse/pose/estimate', Vector3, self.pose_callback)
		rospy.Subscriber('/pacmouse/mode/set_plan_mode', String, self.mode_callback)
		rospy.Subscriber('/pacmouse/maze', Maze, self.maze_callback)
		rospy.Subscriber('/pacmouse/goal', Vector3, self.set_goal_for_testing)
		rospy.spin()

	def plan(self):
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


	def pose_callback(self, msg):
		self.pose[0] = msg.x
		self.pose[1] = msg.y
		self.pose[2] = msg.z

		self.plan()

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

	def maze_callback(self, msg):
		w = msg.width
		h = msg.height
		m = Maze2(w, h)
		m.h_walls = np.array(msg.h_walls).reshape([w, h+1])
		m.h_walls = np.array(msg.v_walls).reshape([w+1, h])
		self.maze = m

		if self.shortest_path_solving:
			# use dijkstras to solve the pairwise distances when we are in shortest path mode
			self.maze.build_adjacency_matrix(threshold=p.wall_transparency_threshold)
			self.maze.solve()

	def set_goal_for_testing(self, msg):
		self.goal_cell = np.array([msg.x, msg.y]).astype(int)
		print 'Goal cell set to {} {}'.format(*self.goal_cell)


if __name__ == '__main__':
	ros_is_garbage = PlannerNode()
