from matplotlib import pyplot as plt
from maze import Maze
import time
import params as p
import numpy as np
from util import rotate_2d

def lidar_observation_function(Z, x, maze):
    """Computes a likelihood given sensor data and a particle position. A higher
    number means x is more likely to produce an observation similar to Z

    Args:
        Z (1d numpy array): a 6-vector of lidar measurements
        x (1d numpy array): a 6-vector of state (of a particle)
        maze (Maze): the maze

    Returns:
        float: how likely that particle is based on agreement the sensor data, Z
    """
    z_exp = estimate_lidar_returns(x, maze)
    return np.prod(1/(np.abs(z_exp - Z) + 1e-5))

def estimate_lidar_returns_old(pose, maze):
    plot = False

    segment_list = maze_to_segment_list(maze)
    line_list = [line(seg[:2], seg[2:]) for seg in segment_list]
    lidar_list = []
    returns = []
    for lidar_transform in p.lidar_transforms:
        lidar_global_xy = pose[:2] + rotate_2d(lidar_transform[:2], pose[2])
        lidar_global_theta = pose[2] + lidar_transform[2]
        lidar_global_end = rotate_2d([5.,0], lidar_global_theta) + lidar_global_xy
        intersections = [lidar_global_end]
        lidar_line = line(lidar_global_end, lidar_global_xy)
        for l,seg in zip(line_list, segment_list):
            if intersect(lidar_global_xy, lidar_global_end, seg[:2], seg[2:]):
                intersections.append(get_intersection(lidar_line, l))
                if plot: plt.plot(intersections[-1][0], intersections[-1][1],'bo')

        returns.append(np.min(np.sqrt(np.sum((lidar_global_xy[None,:] - np.array(intersections))**2,axis=1))))
        min_index = np.argmin(np.sqrt(np.sum((lidar_global_xy[None,:] - np.array(intersections))**2,axis=1)))
        if plot: plt.plot(intersections[min_index][0],intersections[min_index][1], 'ro')
        lidar_list.append(lidar_global_xy.tolist() + lidar_global_end.tolist())


    if plot:
        print(np.array(returns))
        for seg in segment_list:
            plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'k')
        for seg in lidar_list:
            plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'r')
        plt.show()

    return returns


# NOTE(izzy): I rewrote this function to avoid using line intersections.
# It's about 6 times faster, and from my testing, the two implementations seem to agree.
# there's certainly some performance left to be extracted, but I'll leave that for later if
# we need to
def estimate_lidar_returns(pose, maze):
    # Only use this when debugging. If this is true when you
    # run ParticleFilterTests it will break due to matplotlib. 
    plot = False

    c = p.maze_cell_size
    w = p.maze_wall_thickness/2.
    returns = np.zeros(p.lidar_transforms.shape[0])
    for lidar, lidar_transform in enumerate(p.lidar_transforms):
        lidar_global_xy = pose[:2] + rotate_2d(lidar_transform[:2], pose[2])
        lidar_global_theta = pose[2] + lidar_transform[2]
        lidar_global_vector = rotate_2d([1,0], lidar_global_theta) # x, y
        move_right, move_up = lidar_global_vector > 0 # boolean vector

        # get the coordinate of where the lidar line will hit the walls
        h_wall_y_coords = np.arange(0,maze.height+1) * c
        v_wall_x_coords = np.arange(0,maze.width+1) * c

        # calculate the distance from the lidar to those points
        x_dists_to_v_walls = v_wall_x_coords - lidar_global_xy[0]
        y_dists_to_h_walls = h_wall_y_coords - lidar_global_xy[1]

        # only take the walls that are in front of the lidar
        x_dists_to_v_walls = x_dists_to_v_walls[np.sign(x_dists_to_v_walls) == np.sign(lidar_global_vector[0])]
        y_dists_to_h_walls = y_dists_to_h_walls[np.sign(y_dists_to_h_walls) == np.sign(lidar_global_vector[1])]

        # get the other coordinates
        x_dists_to_h_walls = y_dists_to_h_walls/lidar_global_vector[1]*lidar_global_vector[0]
        y_dists_to_v_walls = x_dists_to_v_walls/lidar_global_vector[0]*lidar_global_vector[1]

        # and pair them off and shift back to global coordinates
        h_wall_hit_coords = np.vstack([x_dists_to_h_walls, y_dists_to_h_walls]).T + lidar_global_xy[None,:]
        v_wall_hit_coords = np.vstack([x_dists_to_v_walls, y_dists_to_v_walls]).T + lidar_global_xy[None,:]

        # convert coordinates to x,y indices of the walls
        h_wall_hit_indices = np.floor(h_wall_hit_coords/c).astype(int)
        v_wall_hit_indices = np.floor(v_wall_hit_coords/c).astype(int)

        # conver x,y indices to global array indices
        h_wall_hit_indices = (maze.height+1) * h_wall_hit_indices[:,0] + h_wall_hit_indices[:,1]
        v_wall_hit_indices = maze.height * v_wall_hit_indices[:,0] + v_wall_hit_indices[:,1]

        # only consider indices inside the array
        h_wall_hit_indices = np.clip(h_wall_hit_indices, 0, maze.h_walls.size-1)
        v_wall_hit_indices = np.clip(v_wall_hit_indices, 0, maze.v_walls.size-1)

        # look up on the maze by index where the walls are, and then only take those intersection coordinates
        h_wall_hit_coords = h_wall_hit_coords[maze.h_walls[h_wall_hit_indices] < 1,:]
        v_wall_hit_coords = v_wall_hit_coords[maze.v_walls[v_wall_hit_indices] < 1,:]

        # compute the distances to each intersection coordinate
        dists = np.hstack([np.linalg.norm(h_wall_hit_coords - lidar_global_xy, axis=1),
                           np.linalg.norm(v_wall_hit_coords - lidar_global_xy, axis=1)])

        returns[lidar] = np.min(dists) if dists.size else -1

        if plot:
            lidar_end = lidar_global_xy + lidar_global_vector*5
            plt.plot((lidar_global_xy[0], lidar_end[0]), (lidar_global_xy[1],lidar_end[1]), 'r')
            
            plt.scatter(*h_wall_hit_coords.T)
            plt.scatter(*v_wall_hit_coords.T)

    if plot:
        plot_segment_list(plt, maze_to_segment_list(maze))
        plt.show()

    return returns


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
    c = p.maze_cell_size
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

def plot_segment_list(plt, segment_list):
    segment_list
    for seg in segment_list:
        plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'k')

if __name__ == '__main__':
    m = Maze(16,16)
    m.build_wall_matrices()
    segment_list = maze_to_segment_list(m)
    pose = [0.168 * 4.2, 0.168 * 3.2, np.pi/4]

    start = time.time()
    old_ans =  estimate_lidar_returns_old(pose, m)
    duration = time.time() - start
    print('OLD: {} seconds\t{}'.format(duration, old_ans))

    start = time.time()
    new_ans =  estimate_lidar_returns(pose, m)
    duration = time.time() - start
    print('NEW: {} seconds\t{}'.format(duration, new_ans))

    print np.sum(np.abs(old_ans - new_ans))
