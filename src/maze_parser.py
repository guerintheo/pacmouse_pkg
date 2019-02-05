#!/usr/bin/python3
import numpy as np
import argparse

def adj_to_ascii(adj_matrix, height, width):
    """ Returns the ascii representation of an adjacency matrix

        Args:
            adj_matrix (2d numpy array): adjacency graph representing a maze
            height (int): height (i.e. rows) of maze, in number of cells 
            width (int): width (i.e. columns) of maze, in number of cells)

        Returns:
            string: printable ascii representation of maze
    """
    
    rows = height
    cols = width
    output = '%d,%d\n' % (rows, cols)
    cap = ''.join(['+---' for _ in range(cols)]) + '+\n'
    output += cap
    for r in range(rows):
        lines = ['|' if not adj_matrix[r*cols + c][r*cols + c + 1] else ' ' for c in range(cols-1)]
        output += '|   ' + '   '.join(lines) + '   |\n'
        if r < rows - 1:
            for c in range(cols):
                output += '+---' if not adj_matrix[r*cols + c][(r+1)*cols + c] else '+   '
            output += '+\n'
    output += cap
    return output


def parse_maze_file(fn):
    """ Reads the ascii maze file given in fn and parses into numpy adjacency graph

        Args:
            fn (str): path to ascii maze file (note the maze format's first line
                      must be: <rows>,<cols> which differs from previous format of
                      <cols>,<rows>

        Returns:
            (adj_matrix, rows, cols)
            adj_matrix: adjacency matrix representing the maze
            rows: number of rows in the maze
            cols: number of columns in the maze
    """

    with open(fn,'r') as fo:
        lines = fo.readlines()

    try:
        (maze_rows, maze_cols) = map(int, lines[0].split(','))
    except:
        print('Invalid Maze Format in line 1')

    maze = [list(textrow) for textrow in lines[1:]]

    n_cells = maze_rows * maze_cols
    adj_matrix = np.zeros((n_cells, n_cells))
    for row in range(maze_rows):
        text_row = 2*row + 1
        for col in range(maze_cols):
            text_column = 4*col + 1
            
            if row < (maze_rows-1):
                # want to make sure we aren't in last row
                if ''.join(maze[text_row+1][text_column:text_column+3]) != '---':
                    adj_matrix[row*maze_cols + col][(row+1)*maze_cols + col] = 1

            if col < (maze_cols-1):
                #make sure we aren't in last column
                if maze[text_row][text_column+3] != '|':
                    adj_matrix[row*maze_cols + col][row*maze_cols + col + 1] = 1

    adj_matrix += adj_matrix.transpose()
    adj_matrix += np.eye(n_cells)

    return (adj_matrix, maze_rows, maze_cols)

def main():
    parser = argparse.ArgumentParser('Tests the ascii <-> adjacency graph functionality')
    parser.add_argument('fn', type=str, help='Name of ascii file to demonstrate')
    args = parser.parse_args()
    (adj_matrix, maze_rows, maze_cols) = parse_maze_file(args.fn)
    asc = adj_to_ascii(adj_matrix,maze_rows,maze_cols)
    print(asc)

if __name__ == '__main__':
    main()

