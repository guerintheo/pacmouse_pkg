import os
import sys

import pacmouse_pkg.src.params as p
from pacmouse_pkg.src.utils.maze import Maze2

def load_backup(back):
	mazes = list_mazes()
	m = Maze2()

	if 0 < back <= len(mazes):
		i = mazes[-back]
		maze_file = num_to_maze(i)
		print 'maze_backup loading {}'.format(maze_file)
		m.load(maze_file)

	clear_mazes()
	print 'maze_backup reset'
	return m

def clear_mazes():
	return[os.remove(p.maze_backup_path + '/' + f) for f in os.listdir(p.maze_backup_path) if f.endswith('.maze')]

def list_mazes():
	maze_files = os.listdir(p.maze_backup_path)
	numbers = []
	for maze_file in maze_files:
		try:
			maze_num = int(maze_file.split('.')[0])
			numbers.append(maze_num)
		except:
			print 'maze_backup failed to parse {}'.format(maze_file)

	return sorted(numbers)

def save_backup(maze, i):
	maze.save(num_to_maze(i))


def num_to_maze(i):
	return p.maze_backup_path + '/' + str(i) + '.maze'

if __name__ == '__main__':
	for i in range(10):
		m = Maze2()
		save_backup(m, i)

	print load_backup(2)