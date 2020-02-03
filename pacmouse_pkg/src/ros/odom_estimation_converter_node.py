#!/usr/bin/python
# odom_estimation_converter.py -- Converts gazebo Odometry messages to PoseStamped

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler

odom_x = 0
odom_y = 0
odom_t = 0

def odom_callback(msg):
    global odom_x
    global odom_y
    global odom_t

    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    (r,p,y) = euler_from_quaternion(orientation_list)
    odom_t = y

def convert_odom():
    msg = PoseStamped()
    msg.pose.position.x = odom_x
    msg.pose.position.y = odom_y
    
    quat = quaternion_from_euler(0,0,odom_t)
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    return msg


def main():
    rospy.init_node('odom_estimation_converter')
    odom_sub = rospy.Subscriber('/mouse_diff_drive_controller/odom', Odometry, odom_callback)
    pose_pub = rospy.Publisher('/pose_est', PoseStamped, queue_size=1)

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pose_msg = convert_odom()
        pose_pub.publish(pose_msg)
        loop_rate.sleep()

if __name__ == '__main__':
    main()
