import numpy as np
import time
import os
# only load matplotlib if we are not on the pi
if os.path.expanduser("~") != '/home/pi':
    from matplotlib import pyplot as plt

from pacmouse_pkg.src.utils.maze import Maze, Maze2
import pacmouse_pkg.src.params as p
from pacmouse_pkg.src.utils.math_utils import *

def lidar_observation_function_hyperbolic(Z, x, maze):
    """Computes a likelihood given sensor data and a particle position. A higher
    number means x is more likely to produce an observation similar to Z

    Args:
        Z (1d numpy array): a 6-vector of lidar measurements
        x (1d numpy array): a 6-vector of state (of a particle)
        maze (Maze): the maze

    # Returns:
        float: how likely that particle is based on agreement the sensor data, Z
    """
    z_exp = estimate_lidar_returns(x, maze)
    return np.prod(1/(np.abs(z_exp - Z) + 1e-5))

def lidar_observation_function_hyperbolic_multi(Z, xs, maze):
    """Computes a likelihood given sensor data and a particle position. A higher
    number means x is more likely to produce an observation similar to Z

    Args:
        Z (1d numpy array): a 6-vector of lidar measurements
        x (2d numpy array): an n by 6 of state (of a particle)
        maze (Maze): the maze

    Returns:
        1d numpy array: how likely each particle is based on agreement the sensor data, Z
    """
    z_exp = estimate_lidar_returns_multi(xs, maze)
    return np.prod(1/(np.abs(z_exp - Z) + 1e-5), axis=1)

def lidar_observation_function_gaussian(Z, x, maze):
    z_exp, z_conf = estimate_lidar_returns(x, maze)

    # NOTE(izzy): we take 1-z_conf because z_conf represents how likely there is to be
    # wall at the point where the lidar hits, and we want the variance to be lower if
    # we are more certain that there is a wall at that position.

    # NOTE(aaron): we may need to scale and offset z_conf to avoid zero variance
    # NOTE(aaron): z_conf should never be smaller than the p.wall_transparency_threshold arg
    # to the estimate_lidar_returns model
    base_variance = 0.01
    return np.prod(gaussian(z_exp, Z, base_variance + (1.-z_conf)/100.))

def lidar_observation_function_gaussian_multi(Z, xs, maze):
    z_exp, z_conf = estimate_lidar_returns_multi(xs, maze, return_confidences=True)

    # NOTE(izzy): we take 1-z_conf because z_conf represents how likely there is to be
    # wall at the point where the lidar hits, and we want the variance to be lower if
    # we are more certain that there is a wall at that position.

    # NOTE(aaron): we may need to scale and offset z_conf to avoid zero variance
    # NOTE(aaron): z_conf should never be smaller than the p.wall_transparency_threshold arg
    # to the estimate_lidar_returns model
    base_variance = 0.1
    eps = 1e-3
    # return np.prod(gaussian(z_exp, Z[None,:], base_variance + (1.-z_conf)/100.)+1e-5, axis=1)
    return np.prod(gaussian(z_exp, Z[None,:], base_variance / (z_conf+eps)), axis=1) + eps

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


def debug_plot_poses(debug_plot, poses):
    plt.scatter(*poses[:,:2].T)

def debug_plot_lidar_vecs(debug_plot, lidar_global_xys, lidar_global_vecs, lidar_length=p.lidar_max_dist):
    start_x, start_y = np.moveaxis(lidar_global_xys,2,0)
    end_x, end_y = np.moveaxis(lidar_global_xys+lidar_global_vecs*lidar_length,2,0)
    plt.plot((np.ravel(start_x), np.ravel(end_x)), (np.ravel(start_y), np.ravel(end_y)), c='r', alpha=0.2)

def debug_plot_wall_intersect_coords(debug_plot, wall_intersect_coords, wall_mask=None):
    xs = np.ravel(wall_intersect_coords[:,:,0,:])
    ys = np.ravel(wall_intersect_coords[:,:,1,:])
    if wall_mask is not None:
        mask = np.ravel(wall_mask)
        plt.scatter(xs[mask], ys[mask])
    else:
        plt.scatter(xs, ys)

def debug_plot_lidar_enpoints(debug_plot, lidar_global_vecs, lidar_global_xys, min_dists):
    # [n x m x 2] array
    endpoints = lidar_global_xys + lidar_global_vecs * min_dists[:,:,None]
    xs = np.ravel(endpoints[:,:,0])
    ys = np.ravel(endpoints[:,:,1])
    plt.scatter(xs, ys)

def estimate_lidar_returns_multi(poses, maze, return_confidences=False, debug_plot=None):
    # each pose is a 3-vector [x, y, 3]
    # there are n poses, so poses is an [n x 3] matrix
    # there are m lidars
    # the maze is w x h

    ################## FIND INFORMATION ABOUT LIDAR POSES ##################

    # this is an [m] array
    lidar_local_thetas = p.lidar_transforms[:, 2]

    # this is an [n x 2 x 2] array
    pose_rotation_matricies = np.moveaxis(
                                 np.array([[np.cos(poses[:,2]), -np.sin(poses[:,2])],
                                           [np.sin(poses[:,2]), np.cos(poses[:,2])]]),
                                 2, 0)

    # this is an [m x 2] array
    lidar_local_xys = p.lidar_transforms[:, :2]

    # [n x m x 2] array
    lidar_global_xys = poses[:, None, :2] + np.einsum('nij,mj->nmi', pose_rotation_matricies, lidar_local_xys)

    # this is an [n x m] array
    lidar_global_thetas = poses[:, 2, None] + lidar_local_thetas[None, :]

    # this is an [n x m x 2 x 2] array
    lidar_global_rotation_matrices = np.moveaxis(
                                     np.array([[np.cos(lidar_global_thetas), -np.sin(lidar_global_thetas)],
                                               [np.sin(lidar_global_thetas), np.cos(lidar_global_thetas)]]),
                                     [2,3], [0,1])


    # [n x m x 2] array
    lidar_global_vecs = np.einsum('nmij,j->nmi', lidar_global_rotation_matrices, np.array([1,0]))
    lidar_global_directions = np.sign(lidar_global_vecs)

    if debug_plot: debug_plot_poses(debug_plot, poses)
    if debug_plot: debug_plot_lidar_vecs(debug_plot, lidar_global_xys, lidar_global_vecs)
    
    # v_num_observable_walls = np.min(np.floor(p.lidar_max_dist * 2/p.maze_cell_size) + 2, maze.width)
    # h_num_observable_wals = np.min(np.floor(p.lidar_max_dist * 2/p.maze_cell_size) + 2, maze.height)

    ################## FIND VERTICAL WALL INTERSECTIONS ##################
    # [w+1] the global x coordinates of all the sets vertical walls
    # v_wall_x_coords = np.arange(0, v_num_observable_walls) * p.maze_cell_size + 
    v_wall_x_coords = np.arange(0,maze.width+1) * p.maze_cell_size



    # [n x m x w+1] # calculate the x and y distances from each lidar to each set of vertical walls
    x_dists_to_v_walls = v_wall_x_coords[None, None, :] - lidar_global_xys[:,:,0,None]
    y_dists_to_v_walls = x_dists_to_v_walls/lidar_global_vecs[:,:,0,None]*lidar_global_vecs[:,:,1,None]

    # [n x m x w+1] # create a mask of which wall-intersections are in front of the lidars
    v_wall_in_front_mask = np.equal(np.sign(x_dists_to_v_walls), lidar_global_directions[:,:,0, None])

    # [n x m x 2 x w+1] take the individual x and y distances from lidar to wall, and shift them by the lidar
    # position to get the global vertical wall position for each lidar intersection. 
    v_wall_intersect_coords_local = np.moveaxis(np.stack([x_dists_to_v_walls, y_dists_to_v_walls]),0,2)
    v_wall_intersect_coords = v_wall_intersect_coords_local + lidar_global_xys[:,:,:,None]
    
    # [n x m x w+1] create a mask to remember which v wall intersections are within the maze
    v_wall_in_maze_mask = np.logical_and(v_wall_intersect_coords[:,:,1,:] > 0,
                                         v_wall_intersect_coords[:,:,1,:] < maze.height*p.maze_cell_size)

    # [n x m x 2 x w+1] Then divide by the cell size to get the index of each wall in the maze.v_wall array
    v_wall_intersect_indices = np.floor(v_wall_intersect_coords/p.maze_cell_size).astype(int)
    # we need to mask the indices to make sure we are looking up on valid maze.v_wall indices (not outside the maze)
    v_wall_intersect_indices = v_wall_intersect_indices * v_wall_in_maze_mask[:,:,None,:]

    # [n x m x w+1] # do a lookup on the maze to see whether or not a wall exists at each of these x,y indices
    v_wall_hit_confidences = maze.v_walls[v_wall_intersect_indices[:,:,0,:],
                                          v_wall_intersect_indices[:,:,1,:]]

    # [n x m x w+1] only consider the hits which occur on a sufficiently confident wall
    v_wall_hit_mask = v_wall_hit_confidences > p.wall_transparency_threshold

    # [n x m x w+1] a mask of which potential wall intersections are valid
    v_wall_mask = v_wall_hit_mask * v_wall_in_maze_mask * v_wall_in_front_mask

    # if debug_plot: debug_plot_wall_intersect_coords(debug_plot, v_wall_intersect_coords, v_wall_mask)

    ################## FIND HORIZONTAL WALL INTERSECTIONS ##################
    # [h+1] the global y coordinates of all the sets horizontal walls
    h_wall_y_coords = np.arange(0,maze.width+1) * p.maze_cell_size

    # [n x m x h+1] # calculate the x and y distances from each lidar to each set of horizontal walls
    y_dists_to_h_walls = h_wall_y_coords[None, None, :] - lidar_global_xys[:,:,1,None]
    x_dists_to_h_walls = y_dists_to_h_walls/lidar_global_vecs[:,:,1,None]*lidar_global_vecs[:,:,0,None]

    # [n x m x h+1] # create a mask of which wall-intersections are in front of the lidars
    h_wall_in_front_mask = np.equal(np.sign(x_dists_to_h_walls), lidar_global_directions[:,:,0, None])

    # [n x m x 2 x h+1] take the individual x and y distances from lidar to wall, and shift them by the lidar
    # position to get the global horizontal wall position for each lidar intersection. 
    h_wall_intersect_coords_local = np.moveaxis(np.stack([x_dists_to_h_walls, y_dists_to_h_walls]),0,2)
    h_wall_intersect_coords = h_wall_intersect_coords_local + lidar_global_xys[:,:,:,None]
    
    # [n x m x h+1] create a mask to remember which horizontal wall intersections are within the maze
    h_wall_in_maze_mask = np.logical_and(h_wall_intersect_coords[:,:,0,:] > 0,
                                         h_wall_intersect_coords[:,:,0,:] < (maze.width)*p.maze_cell_size)

    # [n x m x 2 x h+1] Then divide by the cell size to get the index of each wall in the maze.h_wall array
    h_wall_intersect_indices = np.floor(h_wall_intersect_coords/p.maze_cell_size).astype(int)
    # we need to mask the indices to make sure we are looking up on valid maze.h_wall indices (not outside the maze)



    h_wall_intersect_indices = h_wall_intersect_indices * h_wall_in_maze_mask[:,:,None,:]

    # [n x m x h+1] # do a lookup on the maze to see whether or not a wall exists at each of these x,y indices
    h_wall_hit_confidences = maze.h_walls[h_wall_intersect_indices[:,:,0,:],
                                          h_wall_intersect_indices[:,:,1,:]]

    # [n x m x h+1] only consider the hits which occur on a sufficiently confident wall
    h_wall_hit_mask = h_wall_hit_confidences > p.wall_transparency_threshold

    # [n x m x h+1] a mask of which potential wall intersections are valid
    h_wall_mask = h_wall_hit_mask * h_wall_in_maze_mask * h_wall_in_front_mask

    # if debug_plot: debug_plot_wall_intersect_coords(debug_plot, h_wall_intersect_coords, h_wall_mask)

    ################## GET THE CLOSEST WALL INTERSECTION ##################

    # [n x m w+1] and [n x m h+1] respectively. distances of zero correspond to invalid
    v_wall_intersect_distances = np.linalg.norm(v_wall_intersect_coords_local, axis=2) * v_wall_mask
    h_wall_intersect_distances = np.linalg.norm(h_wall_intersect_coords_local, axis=2) * h_wall_mask

    # create masked arrays to block off the zeros and find the nonzero min
    v_intersect_min_distance = np.ma.masked_equal(v_wall_intersect_distances, 0, copy=False).min(axis=2)
    h_intersect_min_distance = np.ma.masked_equal(h_wall_intersect_distances, 0, copy=False).min(axis=2)

    # return the shorter of the vertical and horizontal min distances (excluding nans)
    stacked_min_distance = np.array([v_intersect_min_distance, h_intersect_min_distance])
    min_dists = np.nanmin(stacked_min_distance, axis=0)
    choose_horizontal = np.nanargmin(stacked_min_distance, axis=0)

    # subtract off the maze wall thickness
    vertical_dist_to_remove = p.maze_wall_thickness/2./np.abs(lidar_global_vecs[:,:,0])
    horizontal_dist_to_remove = p.maze_wall_thickness/2./np.abs(lidar_global_vecs[:,:,1])

    min_dists -= choose_horizontal * horizontal_dist_to_remove
    min_dists -= (1-choose_horizontal) * vertical_dist_to_remove
    less_than_sus_dist = min_dists < p.lidar_sus_dist
    min_dists = np.clip(min_dists, 0, p.lidar_max_dist)

    if debug_plot: debug_plot_lidar_enpoints(debug_plot, lidar_global_vecs, lidar_global_xys, min_dists)

    if return_confidences:
        v_intersect_argmin_distance = np.ma.masked_equal(v_wall_intersect_distances, 0, copy=False).argmin(axis=2)
        h_intersect_argmin_distance = np.ma.masked_equal(h_wall_intersect_distances, 0, copy=False).argmin(axis=2)
        min_dist_h_wall_hit_confidences = np.take_along_axis(h_wall_hit_confidences, h_intersect_argmin_distance[:,:,None], axis=2)[:,:,0]
        min_dist_v_wall_hit_confidences = np.take_along_axis(v_wall_hit_confidences, v_intersect_argmin_distance[:,:,None], axis=2)[:,:,0]
        confidences = choose_horizontal * min_dist_h_wall_hit_confidences + (1-choose_horizontal) * min_dist_v_wall_hit_confidences * less_than_sus_dist
        return min_dists, confidences

    else:
        return min_dists


def debug_plot_mark_intersections_vertical(debug_plot, x_coords, y_coords, decrement_coeffs, on_vecs_mask):
    num_lidars, num_intersections = on_vecs_mask.shape
    for l in range(num_lidars):
        for i in range(num_intersections):
            if on_vecs_mask[l, i] == 1:
                d = decrement_coeffs[l, i]
                plt.scatter(x_coords[i], y_coords[l, i], color=(1,0,0,d))

def debug_plot_mark_intersections_horizontal(debug_plot, x_coords, y_coords, decrement_coeffs, on_vecs_mask):
    num_lidars, num_intersections = on_vecs_mask.shape

    for l in range(num_lidars):
        for i in range(num_intersections):
            if on_vecs_mask[l, i] == 1:
                d = decrement_coeffs[l, i]
                plt.scatter(x_coords[l,i], y_coords[i], color=(1,0,0,d))

def debug_plot_mark_lidar_endpoints(debug_plot, lidar_global_ends, h_wall_mask, h_centeredness, v_centeredness):
    alpha = h_wall_mask * h_centeredness + (1-h_wall_mask) * v_centeredness
    for l, on_h_wall in enumerate(h_wall_mask):
        plt.scatter(lidar_global_ends[l, 0], lidar_global_ends[l,1], color=(0, on_h_wall, 0, alpha[l]))

def update_walls(pose, lidars, maze, decrement_amount=0.05, increment_amount=0.05, debug_plot=None):
    # there are m lidars
    # the maze is w x h

    lidars = np.clip(lidars, 0, p.lidar_sus_dist)

    c = p.maze_cell_size

    # this is an [m] array
    lidar_local_thetas = p.lidar_transforms[:, 2]

    # this is an [m] array
    lidar_global_thetas = pose[2] + lidar_local_thetas

    # this is an [m x 2 x 2] array
    lidar_global_rotation_matrices = np.moveaxis(
                                     np.array([[np.cos(lidar_global_thetas), -np.sin(lidar_global_thetas)],
                                               [np.sin(lidar_global_thetas), np.cos(lidar_global_thetas)]]),
                                     2, 0)

    # this is an [m x 2] array
    lidar_local_xys = p.lidar_transforms[:, :2]

    # [m x 2] array
    lidar_global_xys = pose[None, :2] + np.einsum('ij,mj->mi', rotation_matrix_2d(pose[2]), lidar_local_xys)

    # [m x 2] array
    lidar_global_vecs = np.einsum('mij,mj->mi', lidar_global_rotation_matrices, np.array([[1,0]]) * lidars[:, None])

    lidar_global_ends = lidar_global_xys + lidar_global_vecs

    lidar_lowers = np.min([lidar_global_xys, lidar_global_ends], axis=0)
    lidar_uppers = np.max([lidar_global_xys, lidar_global_ends], axis=0)

    if debug_plot: debug_plot_poses(debug_plot, pose[None,:])
    if debug_plot: debug_plot_lidar_vecs(debug_plot, lidar_global_xys[None,:,:], lidar_global_vecs[None,:,:])

    ########################## DECREMENT WALLS THAT WE PASS THRU ##########################
    # handle vertical walls first

    # [m]
    x_indices = np.arange(maze.width+1)
    x_coords = x_indices * c
    # [m x w+1] this is a matrix with one row for each of the lidars
    # y = (x - x0)/dx * dy + y0
    y_coords = (x_coords[None,:] - lidar_global_xys[:,0, None])/lidar_global_vecs[:,0, None] *\
               lidar_global_vecs[:,1, None] + lidar_global_xys[:, 1, None]
    y_indices = np.floor(y_coords/c).astype(int)
    y_indices = np.clip(y_indices, 0, maze.height-1)

    # create a mask of the intersection coordates that are actually on the rays of the lidars
    on_vecs_mask = np.sum(np.stack([x_coords[None,:] > lidar_lowers[:,0, None], x_coords[None,:] < lidar_uppers[:,0, None],
                                    y_coords > lidar_lowers[:,1, None], y_coords < lidar_uppers[:,1, None]]), axis=0)
    on_vecs_mask = (on_vecs_mask==4)

    # [m x w+1] decrement the walls less if the lidar doesn't pass through the middle
    centeredness = 1. - np.abs(c/2. - y_coords % c)
    orthogonality = np.abs(np.cos(lidar_global_thetas))
    decrement_coeffs = centeredness * orthogonality[:, None]

    if debug_plot: debug_plot_mark_intersections_vertical(debug_plot, x_coords, y_coords, decrement_coeffs, on_vecs_mask)

    # and subtract from all the corresponding walls
    maze.v_walls[x_indices, y_indices] -= decrement_coeffs * decrement_amount * on_vecs_mask

    # # handle horizontal walls second
    y_indices = np.arange(maze.height+1)
    y_coords = y_indices * c

    # [m x h+1] this is a matrix with one row for each of the lidars
    # x = (y - y0)/dy * dx + x0
    x_coords = (y_coords[None,:] - lidar_global_xys[:,1, None])/lidar_global_vecs[:,1, None] *\
               lidar_global_vecs[:,0, None] + lidar_global_xys[:, 0, None]
    x_indices = np.floor(x_coords/c).astype(int)
    x_indices = np.clip(x_indices, 0, maze.width-1)

    # create a mask of the intersection coordates that are actually on the rays of the lidars
    on_vecs_mask = np.sum(np.stack([x_coords > lidar_lowers[:,0, None], x_coords < lidar_uppers[:,0, None],
                                    y_coords[None,:] > lidar_lowers[:,1, None], y_coords[None,:] < lidar_uppers[:,1, None]]), axis=0)
    # and subtract from all the corresponding walls
    on_vecs_mask = (on_vecs_mask==4)

    # [m x h+1] decrement the walls less if the lidar doesn't pass through the middle
    centeredness = 1. - np.abs(c/2. - x_coords % c)
    orthogonality = np.abs(np.sin(lidar_global_thetas))
    decrement_coeffs = centeredness * orthogonality[:, None]

    if debug_plot: debug_plot_mark_intersections_horizontal(debug_plot, x_coords, y_coords, decrement_coeffs, on_vecs_mask)

    # and subtract from all the corresponding walls
    maze.h_walls[x_indices, y_indices] -= decrement_coeffs * decrement_amount * on_vecs_mask

    ########################## INCREMENT WALLS THAT WE HIT ##########################
    # divide by the cell size
    normalized_ends = lidar_global_ends/c

    end_in_maze_mask = np.prod(np.logical_and(normalized_ends > 0,
                                              normalized_ends < np.array([maze.width, maze.height])[None,:]), axis=1)

    max_lidar_dist_mask = lidars < p.lidar_sus_dist

    # find which walls are closest. this will be a list with 1s if the h walls are closer. 0s for v walls
    h_wall_mask = np.argmin(np.abs(normalized_ends - np.round(normalized_ends)), axis=1)

    # convert coordinates to x,y indices of the walls
    h_indices = np.clip(np.array([np.floor(normalized_ends[:,0]), np.round(normalized_ends[:,1])], dtype=int).T, 0, maze.height-1)
    v_indices = np.clip(np.array([np.round(normalized_ends[:,0]), np.floor(normalized_ends[:,1])], dtype=int).T, 0, maze.width-1)

    # this is 1 when the lidar is centered on the wall. zero when it is at the edge of the wall
    h_centeredness = 1 - 2*np.abs(normalized_ends[:,0]%1 - 0.5)
    v_centeredness = 1 - 2*np.abs(normalized_ends[:,1]%1 - 0.5)

    if debug_plot: debug_plot_mark_lidar_endpoints(debug_plot, lidar_global_ends, h_wall_mask, h_centeredness, v_centeredness)

    # update the maze
    maze.h_walls[h_indices[:,0], h_indices[:,1]] += increment_amount * h_wall_mask * h_centeredness * end_in_maze_mask * max_lidar_dist_mask
    maze.v_walls[v_indices[:,0], v_indices[:,1]] += increment_amount * (1-h_wall_mask) * v_centeredness * end_in_maze_mask * max_lidar_dist_mask

    # make sure the wall probabilities stay between 0 and 1
    maze.h_walls = np.clip(maze.h_walls, 0, 1)
    maze.v_walls = np.clip(maze.v_walls, 0, 1)


def lidar_end_points(pose, lidars):
    R = rotation_matrix_2d(pose[2])
    lidar_global_start = pose[None, :2] + np.dot(R, p.lidar_transforms[:, :2].T).T
    lidar_global_theta = p.lidar_transforms[:,2] + pose[2]
    lidar_global_vecs = rotate_2d_multiple(np.array([lidars, np.zeros_like(lidars)]).T, lidar_global_theta)
    return lidar_global_start + lidar_global_vecs


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
    w = p.maze_wall_thickness/2.
    for x in range(maze.width):
        for y in range(maze.height):
            if x < maze.width - 1:
                if not maze.get_connected([x,y], [x+1,y]):
                    segment_list.append([(x+1)*c-w, y*c-w, (x+1)*c-w, (y+1)*c+w])
                    segment_list.append([(x+1)*c+w, y*c-w, (x+1)*c+w, (y+1)*c+w])
            if y < maze.height - 1:
                if not maze.get_connected([x,y], [x,y+1]):
                    segment_list.append([x*c-w, (y+1)*c-w, (x+1)*c+w, (y+1)*c-w])
                    segment_list.append([x*c-w, (y+1)*c+w, (x+1)*c+w, (y+1)*c+w])

    # inner walls
    segment_list.append([w, w, maze.width*c-w, w])
    segment_list.append([w, w, w, maze.height*c-w])
    segment_list.append([maze.width*c-w,w, maze.width*c-w, maze.height*c-w])
    segment_list.append([w, maze.height*c-w, maze.width*c-w, maze.height*c-w])

    # outer walls
    segment_list.append([-w, -w, maze.width*c+w, -w])
    segment_list.append([-w, -w, -w, maze.height*c+w])
    segment_list.append([maze.width*c+w,-w, maze.width*c+w, maze.height*c+w])
    segment_list.append([-w, maze.height*c+w, maze.width*c+w, maze.height*c+w])
    return segment_list

def plot_segment_list(plt, segment_list):
    segment_list
    for seg in segment_list:
        plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'k')

def test_estimate_lidar_returns_multi():
    maze = Maze2(8,8)
    maze.generate_random_maze()
    # maze.v_walls *= np.random.rand(*maze.v_walls.shape)
    # maze.h_walls *= np.random.rand(*maze.h_walls.shape)

    # # plot the maze
    maze.build_segment_list()
    maze.plot(plt)
    border = p.maze_cell_size
    plt.xlim(-border, maze.width * p.maze_cell_size+border)
    plt.ylim(-border, maze.height * p.maze_cell_size+border)

    # generate poses
    n = 3
    poses = np.array([np.random.rand(n)*maze.width * p.maze_cell_size,
                      np.random.rand(n)*maze.height * p.maze_cell_size,
                      np.random.rand(n)*np.pi*2]).T

    lidars, confidences = estimate_lidar_returns_multi(poses, maze, return_confidences=True, debug_plot=plt)

    plt.show()

def test_update_walls():
    maze = Maze2(4,4)
    maze.generate_random_maze()

    # # plot the maze
    maze.build_segment_list()
    maze.plot(plt)
    border = p.maze_cell_size
    plt.xlim(-border, maze.width * p.maze_cell_size+border)
    plt.ylim(-border, maze.height * p.maze_cell_size+border)

    # generate poses
    n = 1
    pose = np.array([(np.random.randint(maze.width) + 0.5)* p.maze_cell_size,
                      (np.random.randint(maze.height) + 0.5)* p.maze_cell_size,
                      np.random.rand()*np.pi*2])

    lidars = estimate_lidar_returns_multi(pose[None,:], maze, debug_plot=False)[0]

    update_walls(pose, lidars, maze, debug_plot=plt)

    plt.show()

if __name__ == '__main__':
    test_estimate_lidar_returns_multi()
    # test_update_walls()


    # m = Maze(16,16)
    # m.build_wall_matrices()
    # segment_list = maze_to_segment_list(m)
    # pose = [0.168 * 4.2, 0.168 * 3.2, np.random.rand() * np.pi/2]

    # start = time.time()
    # old_ans =  estimate_lidar_returns_old(pose, m)
    # duration = time.time() - start
    # print('OLD: {} seconds\t{}'.format(duration, old_ans))

    # start = time.time()
    # new_ans =  estimate_lidar_returns(pose, m)
    # duration = time.time() - start
    # print('NEW: {} seconds\t{}'.format(duration, new_ans))

    # total_error = np.sum(np.abs(old_ans - new_ans))
    # print 'Total Error:', total_error
    # if total_error > 1e-6:
    #     estimate_lidar_returns(pose, m, True)
