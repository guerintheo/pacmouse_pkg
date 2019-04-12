import rospy
import numpy as np

from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from pacmouse_pkg.msg import Maze

from pacmouse_pkg.src.utils.maze import Maze2
import pacmouse_pkg.src.params as p

class PlannerNode:
	def __init__(self):
		rospy.init_node('planner_node')
		rospy.Subscriber('/pacmouse/pose/mocap', Vector3, self.pose_callback)
		rospy.Subscriber('/pacmouse/mode/set_plan_mode', String, self.mode_callback)
		rospy.Subscriber('/pacmouse/maze', Maze, self.maze_callback)
		self.plan_publisher = rospy.Publisher('/pacmouse/plan', Vector3, queue_size=1)

		self.pose = np.zeros(3)
		self.prev_plan = np.zeros(3)
		self.shortest_path_solving = False
		self.maze = None
		self.goal_cell = None
		
		rospy.spin()

	def plan_shortest_path(self):
		current_cell = np.floor(self.pose/p.maze_cell_size)

		# only plan if we are within a certain distance of the target of our previous plan
		if np.linalg.norm(self.pose[:2] - self.prev_plan[:2]) < p.distance_to_cell_center_for_replan:
			plan = self.maze.get_path(current_cell, self.goal_cell)
			print 'Replanning!'
			if len(plan) < 2:
				print 'The plan is too short... I think we made it??!?'
			else:
				target_index = plan[1]
				target_cell = np.array([target_index % self.maze.width, np.floor(target_index/self.maze.width)])
				target_coord = (target_cell + 0.5) * p.maze_cell_size
				msg = Vector3()
				msg.x = target_cell[0]
				msg.y = target_cell[1]
				self.plan_publisher.publish(msg)


	def pose_callback(self, msg):
		self.pose[0] = msg.x
		self.pose[1] = msg.y
		self.pose[2] = msg.z

		if self.shortest_path_solving: self.plan_shortest_path()
		else: self.plan_tremaux()
		
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

if __name__ == '__main__':
	ros_is_garbage = PlannerNode()