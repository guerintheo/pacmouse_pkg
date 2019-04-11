#!/usr/bin/python
import rospy
from geometry_msgs.msg import Vector3
from pacmouse_pkg.msg import Drive
from pacmouse_pkg.src.estimation_control.dynamics import vel_and_psi_dot_from_wheel_vels
import numpy as np

class OdomGroundTruthCompare(object):
    """
    Compare ground truth position to the expected position from our motion model
    informed by the encoders, which are publishing wheel angular velocities.
    """
    
    def __init__(self):
        rospy.init_node('odom_ground_truth_compare')
        rospy.Subscriber('/pacmouse/pose/mocap', Vector3, self.cb_mocap)
        rospy.Subscriber('/pacmouse/encoders/velocity', Drive, self.cb_encoders)
        self.mocap_predict_pub = rospy.Publisher('/pacmouse/mocap/predicted_velocity', Vector3, queue_size=1)
        self.encoders_predict_pub = rospy.Publisher('/pacmouse/encoders/predicted_velocity', Vector3, queue_size=1)
        
        self.received_mocap = False
        self.received_encoders = False
        self.prev_mocap_yaw = 0
        self.prev_mocap_pos = [0, 0]
        self.prev_mocap_time = rospy.Time.now()
        self.prev_encoders_time = rospy.Time.now()
        self.spin()
        
    def cb_mocap(self, data):
        """data is a Vector3 msg"""
        self.received_mocap = True
        new_mocap_yaw = data.z
        curr_time = rospy.Time.now()
        dt = curr_time - self.prev_mocap_time
        mocap_yaw_rate = (new_mocap_yaw - self.prev_mocap_yaw)/dt
        self.prev_mocap_yaw = new_mocap_yaw
        self.prev_mocap_time = curr_time
        
        new_mocap_pos = [data.x, data.y]
        dist_travelled = np.hypot(new_mocap_pos[0] - self.prev_mocap_pos[0], new_mocap_pos[1] - self.prev_mocap_pos[1])
        self.prev_mocap_pos = new_mocap_pos
        vel = dist_travelled/dt
        
        new_msg = Vector3()
        new_msg.x = vel
        new_msg.y = mocap_yaw_rate
        self.mocap_predict_pub.publish(new_msg)
        
    def cb_encoders(self, data):
        """data is a Drive msg"""
        self.received_encoders = True
        vel_and_yaw_rate = vel_and_psi_dot_from_wheel_vels([data.L, data.R])
        new_msg = Vector3()
        new_msg.x = vel_and_yaw_rate[0]  # forward velocity
        new_msg.y = vel_and_yaw_rate[1]  # yaw rate
        self.encoders_predict_pub.publish(new_msg)
        
        
    def spin(self):
        
        rospy.spin()
        # r = rospy.Rate(30)
        # while not rospy.is_shutdown():
        #     pass
        #     r.sleep()
        

if __name__ == '__main__':
    odom = OdomGroundTruthCompare()