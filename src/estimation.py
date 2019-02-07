from map import Maze
import time
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

def plot_lidar_returns(pose, maze):
    from matplotlib import pyplot as plt
    segment_list = maze_to_segment_list(maze)
    line_list = [line(seg[:2], seg[2:]) for seg in segment_list]
    lidar_list = []
    returns = []
    for lidar_transform in p.lidar_transforms:
        lidar_global_xy = pose[:2] + rotate2d(lidar_transform[:2], pose[2])
        lidar_global_theta = pose[2] + lidar_transform[2]
        lidar_global_end = rotate2d([5.,0], lidar_global_theta) + lidar_global_xy
        intersections = [lidar_global_end]
        lidar_line = line(lidar_global_end, lidar_global_xy)
        for l,seg in zip(line_list, segment_list):
            if intersect(lidar_global_xy, lidar_global_end, seg[:2], seg[2:]):
                intersections.append(get_intersection(lidar_line, l))
                plt.plot(intersections[-1][0], intersections[-1][1],'bo')

        returns.append(np.min(np.sqrt(np.sum((lidar_global_xy[None,:] - np.array(intersections))**2,axis=1))))
        min_index = np.argmin(np.sqrt(np.sum((lidar_global_xy[None,:] - np.array(intersections))**2,axis=1)))
        plt.plot(intersections[min_index][0],intersections[min_index][1], 'ro')
        lidar_list.append(lidar_global_xy.tolist() + lidar_global_end.tolist())


    print np.array(returns)
    for seg in segment_list:
        plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'k')
    for seg in lidar_list:
        plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'r')
    plt.show()

# return true of the points A,B,C are aranged in a counterclockwise orientation
def ccw(A, B, C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A, B, C, D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

# get the slope-point form of a line through a pair of points
def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

# find the intersection of two lines
def get_intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False


# generate a list of line segments corresponding to the walls of the maze
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

def plot_segment_list(segment_list):
    from matplotlib import pyplot as plt
    segment_list
    for seg in segment_list:
        plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'k')
    plt.show()

if __name__ == '__main__':
    m = Maze(16,16)
    segment_list = maze_to_segment_list(m)
    # plot_segment_list(segment_list)
    pose = [0.084, 0.084, np.pi/4]
    plot_lidar_returns(pose, m)
