from map import Maze
import params as p
import numpy as np

# def get_2d_rotation(theta):
#     return np.array([[np.cos(theta), -np.sin(theta)],
#                      [np.sin(theta), np.cos(theta)]])

# class Pose:
#     def __init__(self, t=np.zeros(2), r=np.zeros(2,2)):
#         assert t.size == r.shape[0] == r.shape[1]
#         self.t = t
#         self.rotation = r
#         self.dim = self.t.size

#     def __mul__(self, other):
#         # TODO: apply to another pose
#         pass

#     def inverse(self):
#         # TODO: get the inverse
#         pass

# TODO:
# * settle on a representation for the robot pose -- rotation, translation (2D?) 
# * choose the origin for the maze coordinate system
# * write a func to get estiamted lidar returns (distributions given a pose and a maze)
# * 

def rotate2d(coord, theta):
    r = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    return np.dot(r, coord)

def estimate_lidar_returns(pose, maze):
    segment_list = maze_to_segment_list(maze)
    for lidar_transform in p.lidar_transforms:
        lidar_global_xy = pose[:2] + rotate2d(lidar_transform[:2], pose[2])
        lidar_global_theta = pose[2] + lidar_transform[2]
        lidar_global_end = rotate2d([5.,0], lidar_global_theta)
        segment_list.append(lidar_global_xy.tolist() + lidar_global_end.tolist())

    from matplotlib import pyplot as plt

    for seg in segment_list[:-6]:
        plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'k')
    for seg in segment_list[-6:]:
        plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'r')
    plt.show()


def maze_to_segment_list(maze):
    segment_list = []
    c = p.maze_inner_size
    for x in range(maze.width):
        for y in range(maze.height):
            if x < maze.width - 1:
                if not maze.get_connected([x,y], [x+1,y]):
                    segment_list.append([(x+1)*c, y*c, (x+1)*c, (y+1)*c])
            if y < maze.height - 1:
                if not maze.get_connected([x,y], [x,y+1]):
                    segment_list.append([x*c, (y+1)*c, (x+1)*c, (y+1)*c])


    segment_list.append([0, 0, maze.width*c, 0])
    segment_list.append([0, 0, 0, maze.height*c])
    segment_list.append([maze.width*c,0, maze.width*c, maze.height*c])
    segment_list.append([0, maze.height*c, maze.width*c, maze.height*c])
    return segment_list

def test_generate_segment_list():
    # verify that generate segment list works as intended
    from matplotlib import pyplot as plt
    m = Maze(6,5)
    print m
    segment_list = maze_to_segment_list(m)
    for seg in segment_list:
        plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'k')
    plt.show()

if __name__ == '__main__':
    # test_generate_segment_list()
    pose = [0.1,0.2, np.pi/4]
    m = Maze(16,16)
    estimate_lidar_returns(pose, m)
