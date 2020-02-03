#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('test_pose_rviz', anonymous=True)
    pose_pub = rospy.Publisher('/pacmouse/pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(30) # rate of 30 Hz
    angle = 0
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    # TF data broadcaster makes it so that RViz "Global Status", "Fixed Frame"
    # does not show a warning of no TF data
    broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        angle += 0.1
        angle %= 2*math.pi
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = math.cos(angle)
        pose.pose.position.y = math.sin(angle)
        quat = tf.transformations.quaternion_from_euler(0, 0, angle+math.pi/2)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        print pose
        broadcaster.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
                          tf.transformations.quaternion_from_euler(0, 0, 0),
                          rospy.Time.now(),
                          "robot",
                          "world")
        pose_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    main()
