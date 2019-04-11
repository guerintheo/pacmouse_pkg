#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from pyquaternion import Quaternion
import numpy as np

class MocapRepublisher(object):
    
    def __init__(self):
        rospy.init_node('mocap_republisher')
        self.pose_repub = rospy.Publisher('/pacmouse/pose/mocap', Vector3, queue_size=1)
        rospy.Subscriber('/vrpn_client_node/pacmouse/pose', PoseStamped, self.cb_pose)
        rospy.spin()
        
    def cb_pose(self, data):
        """Process mocap pose message and republish in a simpler format."""
        repub_msg = Vector3()
        repub_msg.x = data.pose.position.x
        repub_msg.y = data.pose.position.y
        quat = Quaternion([data.pose.orientation.w, data.pose.orientation.x,
                           data.pose.orientation.y, data.pose.orientation.z])
        start_vec = [1.,0.,0.]
        rot_vec = quat.rotate(start_vec)
        yaw = np.arctan2(rot_vec[1], rot_vec[0])
        repub_msg.z = yaw
        self.pose_repub.publish(repub_msg)


if __name__ == '__main__':
    repub = MocapRepublisher()
        