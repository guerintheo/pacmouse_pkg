#!/usr/bin/env python3


import argparse

from pacmouse_pkg.src.utils.maze_parser import parse_maze_file
import pacmouse_pkg.src.params as p


def make_world_header():
    world_header = '<?xml version="1.0" ?>\n' \
                   '<sdf version="1.5">\n' \
                   '  <world name="TestMazeAutogen">\n' \
                   '    <include>\n' \
                   '      <uri>model://sun</uri>\n' \
                   '    </include>\n' \
                   '    <include>\n' \
                   '      <uri>model://ground_plane</uri>\n' \
                   '    </include>'
    return world_header


def make_world_footer():
    return '  </world>\n</sdf>'


def make_block(pose, name):
    block_txt = '\n    <include>\n      <uri>%s</uri>\n      ' \
                '<name>%s</name>\n      <pose>%f %f %f %f %f %f</pose>\n    ' \
                '</include>\n' % (p.wall_uri, name, pose[0], pose[1], pose[2],
                                  pose[3], pose[4], pose[5])
    return block_txt


def main():
    parser = argparse.ArgumentParser('Utility to turn a maze description into '
                                     'an .sdf that can be launched by gazebo')
    parser.add_argument('input_file', type=str, help="Name of ascii maze file "
                                                     "to convert into sdf")
    parser.add_argument('output_file', type=str, help="Name of .sdf file to "
                                                      "output")
    args = parser.parse_args()
    (adj_matrix, n_x, n_y) = parse_maze_file(args.input_file)

    for r in adj_matrix:
        print(r)

    header = make_world_header()
    block_body = ''
    bix = 1

    max_x = n_x - 1
    max_y = n_y - 1
    eww = p.maze_cell_size  # "Effective Wall Width" - used for a lot of offset
    # calculations

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
                block_body += make_block([eww/2. + ix*eww, 0, p.wall_height/2., 0,0,0], 'block%s' % bix)
                bix += 1
            if ix == 0:
                block_body += make_block([0, iy*eww + eww/2., p.wall_height/2., 0,0,1.5707], 'block%s' % bix)
                bix += 1

            if (not adj_matrix[adj_ix][adj_ix_r]) or (ix == max_x):
                block_body += make_block([(ix+1)*eww, iy*eww + eww/2., p.wall_height/2., 0,0,1.5707], 'block%s' % bix)
                bix += 1

            if (not adj_matrix[adj_ix][adj_ix_u]) or (iy == max_y) :
                block_body += make_block([eww/2. + ix*eww, (iy + 1)*eww, p.wall_height/2., 0,0,0], 'block%s' % bix)
                bix += 1

    footer = make_world_footer()
    full_world = header + block_body + footer
    with open(args.output_file, 'w+') as fo:
        fo.write(full_world)


if __name__ == '__main__':
    main()
