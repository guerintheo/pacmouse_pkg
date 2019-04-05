import subprocess
import sys

base_path = '/home/pi/ros_catkin_ws/src/pacmouse_pkg/src/'

list_of_nodes = ['hardware_interfaces/imu_node.py',
				 'testing/go_to.py']

list_of_processes = []

def launch_nodes():
	for node in list_of_nodes:
		try:
			list_of_processes.append(subprocess.Popen(['python', node]))
		except:
			print 'Failed to launch', node

def kill_nodes():
	for process in list_of_processes:
		process.kill()

	list_of_processes[:] = [] # empty the list of process