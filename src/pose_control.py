#!/usr/bin/python
# pose_control.py -- Node to control the robot's pose by issuing cmd_vel commands
#
# Subscribes to /pose/est and /traj
# Publishes to /cmd_vel

import numpy as np
import argparse

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

estimated_position = np.zeros(2)
estimated_rotation = 0

desired_position = np.zeros(2)
desired_rotation = 0

def pose_est_callback(msg):
    """ Callback for the /pose/est subscriber. Sets estimated_position, estimated_rotation
    """
    global estimated_position
    global estimated_rotation
    estimated_position[0] = msg.pose.position.x
    estimated_position[1] = msg.pose.position.y

    orientation_q = msg.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_,_,estimated_rotation) = euler_from_quaternion(orientation_list)

def pose_desired_callback(msg):
    """ Callback for the /pose/est subscriber. Sets estimated_position, estimated_rotation
    """
    global desired_position
    global desired_rotation
    desired_position[0] = msg.pose.position.x
    desired_position[1] = msg.pose.position.y

    orientation_q = msg.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (r,p,y) = euler_from_quaternion(orientation_list)
    desired_rotation = y


def calc_command():
    """ Calculate the command to get from pose_est to traj

        TODO: define how pose_est and traj are set
    """
    global desired_position
    global estimated_position
    global desired_rotation
    global estimated_rotation
    P_factor = 0.5
    dp = (desired_position - estimated_position) * P_factor
    dr = (desired_rotation - estimated_rotation) * P_factor

    cmd = Twist()
    cmd.linear.x = dp[0]
    cmd.linear.y = dp[1]
    cmd.angular.z = dr
    return cmd


def main():
    rospy.init_node('pose_control')

    pose_est_sub = rospy.Subscriber('/pose_est', PoseStamped, pose_est_callback)
    pose_desired_sub = rospy.Subscriber('/pose_cmd', PoseStamped, pose_desired_callback)
    #cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd_vel_pub = rospy.Publisher('/mouse_diff_drive_controller/cmd_vel', Twist, queue_size=1)

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print('des. rotation: %f' % desired_rotation)
        vel_msg = calc_command()
        cmd_vel_pub.publish(vel_msg)
        loop_rate.sleep()

if __name__ == '__main__':
    main()
