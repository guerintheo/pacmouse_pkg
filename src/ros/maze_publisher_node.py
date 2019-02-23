import rospy
from pacmouse_pkg.src.utils.maze import Maze
from pacmouse_pkg.msg import Maze as MazeMsg


class MazePublisher:
    """
    Class that publishes the adjacency matrix representing the maze.
    """
    
    def __init__(self):
        rospy.init_node('maze_publisher')
        self.maze_pub = rospy.Publisher('/pacmouse/maze', MazeMsg, queue_size=1)
        self.maze_width = 16
        self.maze_height= 16
        self.maze = Maze(width=self.maze_width, height=self.maze_height)
        print(self.maze)
        
    def publish_loop(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            m = MazeMsg()
            m.adj_matrix = self.maze.adj_matrix.flatten()
            self.maze_pub.publish(m)
            rate.sleep()
    
if __name__ == '__main__':
    maze_publisher = MazePublisher()
    maze_publisher.publish_loop()
