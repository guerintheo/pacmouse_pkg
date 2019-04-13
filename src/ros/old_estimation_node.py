#!/usr/bin/python
# estimation_node.py -- Estimates pose given laser returns and map
#
# Subscribes to /laser_scan_#
# Publishes to /pose_est

import numpy as np
import argparse

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import pacmouse_pkg.src.estimation_control.control as control
from pacmouse_pkg.src.estimation_control.estimation import Estimator
from pacmouse_pkg.src.utils.maze_parser import parse_maze_file
from pacmouse_pkg.src.utils.maze import Maze


laser_ranges = np.zeros(6)

cmd_vel = np.zeros(2)

def laser_scan_callback(msg):
    """ Callback for each time we get a laser scan callback
    """
    global laser_ranges
    frame_id = msg.header.frame_id
    ix = int(frame_id.split('_')[-1])
    if ix > 5:
        print('Error, unknown frame id: %d' % ix)
    laser_ranges[ix] = msg.ranges[0]

def cmd_vel_callback(msg):
    global cmd_vel
    cmd_vel[0] = msg.linear.x
    cmd_vel[1] = msg.angular.z


def main():
    rospy.init_node('estimation_node')

    laser_sub_0 = rospy.Subscriber('/laser_scan_0', LaserScan, laser_scan_callback)
    laser_sub_1 = rospy.Subscriber('/laser_scan_1', LaserScan, laser_scan_callback)
    laser_sub_2 = rospy.Subscriber('/laser_scan_2', LaserScan, laser_scan_callback)
    laser_sub_3 = rospy.Subscriber('/laser_scan_3', LaserScan, laser_scan_callback)
    laser_sub_4 = rospy.Subscriber('/laser_scan_4', LaserScan, laser_scan_callback)
    laser_sub_5 = rospy.Subscriber('/laser_scan_5', LaserScan, laser_scan_callback)
    cmd_vel_sub = rospy.Subscriber('/mouse_diff_drive_controller/cmd_vel', Twist, cmd_vel_callback)
    pose_est_pub = rospy.Publisher('/pose_est', PoseStamped, queue_size=1)

    u_sigma = [.001, .001, .05, 1e-4, 1e-4, 1e-4]


    (adj_matrix, size_x, size_y) = parse_maze_file('../../gazebo_worlds/testmaze.txt')
    
    m = Maze(size_x, size_y)
    m.adj_matrix = adj_matrix 
    m.build_wall_matrices()

    N_DIMS = 6

    est_state = np.array([0.5,0.08,0,0,0,0])
    estimator = Estimator(est_state) # initialize the state estimator
    estimator.set_maze(m)              # pass it the maze

    u_sigma = np.array([.005,.005, np.radians(2), 1e-4, 1e-4, 1e-4])
    estimator.u_sigma = u_sigma           # and the noise model


    # ROS :)
    est_state_message = PoseStamped()
    LOOP_RATE = 10
    loop_rate = rospy.Rate(LOOP_RATE)
    while not rospy.is_shutdown():

        estimator.update((laser_ranges, control.mix(cmd_vel)), 1./LOOP_RATE)
        est_state = estimator.state
        est_state_message.pose.position.x = est_state[0]
        est_state_message.pose.position.y = est_state[1]

        quat = quaternion_from_euler(0,0,est_state[2])
        print('Estimated Theta: %f' % est_state[2])

        est_state_message.pose.orientation.x = quat[0]
        est_state_message.pose.orientation.y = quat[1]
        est_state_message.pose.orientation.z = quat[2]
        est_state_message.pose.orientation.w = quat[3]


        pose_est_pub.publish(est_state_message)

        loop_rate.sleep()

if __name__ == '__main__':
    main()
