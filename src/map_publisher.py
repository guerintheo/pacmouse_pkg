import rospy
from map import Maze
from pacmouse_pkg.msg import Map


class MapPublisher:
    """
    Class that publishes the adjacency matrix representing the maze.
    """
    
    def __init__(self):
        rospy.init_node('map_publisher')
        self.map_pub = rospy.Publisher('/pacmouse/map', Map, queue_size=1)
        self.maze_width = 16
        self.maze_height= 16
        self.maze = Maze(self.maze_width, self.maze_height)
        print(self.maze)
        
    def publish_loop(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            m = Map()
            m.adj_matrix = self.maze.adj_matrix.flatten()
            self.map_pub.publish(m)
            rate.sleep()
    
if __name__ == '__main__':
    map_publisher = MapPublisher()
    map_publisher.publish_loop()