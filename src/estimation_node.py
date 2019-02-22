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

import control


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
    rospy.init_node('pose_control')

    laser_sub_0 = rospy.Subscriber('/laser_scan_0', LaserScan, laser_scan_callback)
    laser_sub_1 = rospy.Subscriber('/laser_scan_1', LaserScan, laser_scan_callback)
    laser_sub_2 = rospy.Subscriber('/laser_scan_2', LaserScan, laser_scan_callback)
    laser_sub_3 = rospy.Subscriber('/laser_scan_3', LaserScan, laser_scan_callback)
    laser_sub_4 = rospy.Subscriber('/laser_scan_4', LaserScan, laser_scan_callback)
    laser_sub_5 = rospy.Subscriber('/laser_scan_5', LaserScan, laser_scan_callback)
    cmd_vel_sub = rospy.Subscriber('/mouse_diff_drive_controller/cmd_vel', Twist, cmd_vel_callback)
    pose_est_pub = rospy.Publisher('/pose_est', PoseStamped, queue_size=1)

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        vel_msg = calc_command()
        cmd_vel_pub.publish(vel_msg)
        loop_rate.sleep()

if __name__ == '__main__':
    main()
