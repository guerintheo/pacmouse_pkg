#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Empty
from pyquaternion import Quaternion
import numpy as np

import pacmouse_pkg.src.params as p

class MocapRepublisher(object):

    def __init__(self):
        rospy.init_node('mocap_republisher')
        self.pose_offset = np.array([0., 0., 0.])
        self.curr_mocap_pose = np.array([0., 0., 0.])
        self.repub_msg = Vector3()
        self.pose_repub = rospy.Publisher('/pacmouse/pose/mocap', Vector3, queue_size=1)
        rospy.Subscriber('/vrpn_client_node/pacmouse/pose', PoseStamped, self.cb_pose)
        rospy.Subscriber('/pacmouse/mode/zero_pose', Empty, self.cb_zero_pose)
        self.spin()

    def cb_pose(self, data):
        """Process mocap pose message and republish in a simpler format."""
        self.curr_mocap_pose[0] = data.pose.position.x
        self.curr_mocap_pose[1] = data.pose.position.y
        quat = Quaternion([data.pose.orientation.w, data.pose.orientation.x,
                           data.pose.orientation.y, data.pose.orientation.z])
        start_vec = [1.,0.,0.]
        rot_vec = quat.rotate(start_vec)
        yaw = np.arctan2(rot_vec[1], rot_vec[0])
        self.curr_mocap_pose[2] = yaw

        self.repub_msg.x = self.curr_mocap_pose[0] - self.pose_offset[0]
        self.repub_msg.y = self.curr_mocap_pose[1] - self.pose_offset[1]
        self.repub_msg.z = yaw

    def cb_zero_pose(self, data):
        """Zero the pose estimate."""
        # TODO: Account for heading correctly? Do a transform with the yaw angle?
        print('Zeroing the pose.')
        self.pose_offset[:2] = self.curr_mocap_pose[:2] - p.maze_cell_size/2  # or maze_inner_size if origin is to be inner corner of bottom-left cell

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print('---')
            print(self.repub_msg)
            self.pose_repub.publish(self.repub_msg)
            rate.sleep()


if __name__ == '__main__':
    repub = MocapRepublisher()
