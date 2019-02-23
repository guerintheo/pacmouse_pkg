#!/usr/bin/python3

import argparse
import numpy as np
from pacmouse_pkg.src.utils.maze_parser import parse_maze_file

WALL_URI = 'model://maze_wall'
WALL_THICKNESS = .012

# Note this width really specifyies the interior dimension of the room
# It is going to be shorter than the wall width specified in the wall model
# file, because the model needs to be artificially wider to  prevent gaps
# in the maze wall
WALL_WIDTH = .168

WALL_HEIGHT = .050

def make_world_header():
    world_header = '<?xml version="1.0" ?>\n<sdf version="1.5">\n  <world name="TestMazeAutogen">\n    <include>\n      <uri>model://sun</uri>\n    </include>\n    <include>\n      <uri>model://ground_plane</uri>\n    </include>'
    return world_header

def make_world_footer():
    return '  </world>\n</sdf>'

def make_block(pose, name):
    block_txt = '\n    <include>\n      <uri>%s</uri>\n      <name>%s</name>\n      <pose>%f %f %f %f %f %f</pose>\n    </include>\n' % (WALL_URI, name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
    return block_txt


def main():
    parser = argparse.ArgumentParser('Utility to turn a maze description into an .sdf that can be launched by gazebo')
    parser.add_argument('input_file',type=str,help="Name of ascii maze file to convert into sdf")
    parser.add_argument('output_file', type=str,help="Name of .sdf file to output")
    args = parser.parse_args()
    (adj_matrix, n_x, n_y) = parse_maze_file(args.input_file)

    #test_maze = [ [1, 1, 1], [1, 0, 1], [1, 1, 1]]

    #adj_matrix = np.zeros((9,9))
    #adj_matrix = np.eye(9)


    #adj_matrix[0][1] = 1
    #adj_matrix[1][0] = 1
    #adj_matrix[0][3] = 1
    #adj_matrix[3][0] = 1

    #adj_matrix[1][2] = 1
    #adj_matrix[2][1] = 1
    #adj_matrix[1][4] = 1
    #adj_matrix[4][1] = 1

    #adj_matrix[2][5] = 1
    #adj_matrix[5][2] = 1

    #adj_matrix[3][6] = 1
    #adj_matrix[6][3] = 1

    #adj_matrix[5][8] = 1
    #adj_matrix[8][5] = 1

    #adj_matrix[6][7] = 1
    #adj_matrix[7][6] = 1

    #adj_matrix[7][8] = 1
    #adj_matrix[8][7] = 1

    #adj_matrix = np.zeros((9,9))

    #n_x = 3
    #n_y = 3

    for r in adj_matrix:
        print(r)

    header = make_world_header()
    block_body = ''
    bix = 1

    max_x = n_x - 1
    max_y = n_y - 1
    EWW = WALL_WIDTH + WALL_THICKNESS # "Effective Wall Width" - used for a lot of offset calculations

    for ix in range(n_x):
        for iy in range(n_y):

            adj_ix = iy*n_x + ix
            if ix == max_x:
                adj_ix_r = adj_ix
            else:
                adj_ix_r = adj_ix + 1

            if iy == max_y:
                adj_ix_u = adj_ix
            else:
                adj_ix_u = adj_ix + n_x

            # outside border (half of outside border)
            if iy == 0:
                block_body += make_block([EWW/2. + ix*EWW, 0, WALL_HEIGHT/2., 0,0,0], 'block%s' % bix)
                bix += 1
            if ix == 0:
                block_body += make_block([0, iy*EWW + EWW/2., WALL_HEIGHT/2., 0,0,1.5707], 'block%s' % bix)
                bix += 1

            if (not adj_matrix[adj_ix][adj_ix_r]) or (ix == max_x):
                block_body += make_block([(ix+1)*EWW, iy*EWW + EWW/2., WALL_HEIGHT/2., 0,0,1.5707], 'block%s' % bix)
                bix += 1

            if (not adj_matrix[adj_ix][adj_ix_u]) or (iy == max_y) :
                block_body += make_block([EWW/2. + ix*EWW, (iy + 1)*EWW, WALL_HEIGHT/2., 0,0,0], 'block%s' % bix)
                bix += 1

    footer = make_world_footer()
    full_world = header + block_body + footer
    with open(args.output_file, 'w+') as fo:
        fo.write(full_world)

if __name__ == '__main__':
    main()
