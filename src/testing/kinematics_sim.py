#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('test_pose_rviz', anonymous=True)
    pose_pub = rospy.Publisher('/pacmouse/pose', PoseStamped, queue_size=10)
    x = init_state()
    hz = 30
    rate = rospy.Rate(hz) # rate of 30 Hz
    dt = 1.0/hz
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    # TF data broadcaster makes it so that RViz "Global Status", "Fixed Frame"
    # does not show a warning of no TF data
    broadcaster = tf.TransformBroadcaster()
    # omega_left_seq = [800 + 2000*np.sin(num) for num in np.linspace(0., 10., 1000)]
    omega_left_seq = [2000 for _ in np.linspace(0., 10., 1000)]
    omega_right_seq = [2000 + 200*np.sin(3*num) for num in np.linspace(0., 10., 1000)]
    # omega_right_seq = [800 for _ in np.linspace(0., 10., 1000)]
    i = 0
    control_seq_len = len(omega_left_seq)
    while not rospy.is_shutdown():
        if i < control_seq_len:
            u = [omega_left_seq[i], omega_right_seq[i]]
        else:
            u = [0, 0]
        x = step_state(x, u, dt)  # uses kinematic process model
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x[0]
        pose.pose.position.y = x[1]
        # Add pi/2 to make y-axis correspond to 0 degrees
        # TODO: Adjust kinematic/process model so that  body-frame x-axis points forward so that robot
        # points along global-frame x-axis at 0 degrees?
        quat = tf.transformations.quaternion_from_euler(0, 0, x[2]+np.pi/2)
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
        i += 1

def init_state():
    return np.array([0, 0, 0, 0, 0, 0], dtype=float)

def step_state(x, u, dt):
    x_p = x[0]
    y_p = x[1]
    psi = x[2]
    # v = x[3]
    v = 0.000090280*(u[0] + u[1])
    a = x[4]
    # psi_dot = x[5]
    psi_dot = 0.0015141*(u[1] - u[0])
    # TODO: Address this special case better
    if psi_dot == 0:
        psi_dot = 0.0001
    x_p_new = (1.0/(psi_dot**2))*((v+a*dt)*psi_dot*np.cos(psi+psi_dot*dt) - v*psi_dot*np.cos(psi) -
                a*np.sin(psi + psi_dot*dt) + a*np.sin(psi))
    y_p_new = (1.0/(psi_dot**2))*((v+a*dt)*psi_dot*np.sin(psi+psi_dot*dt) - v*psi_dot*np.sin(psi) +
                a*np.cos(psi + psi_dot*dt) - a*np.cos(psi))
    x_new = x + np.array([x_p_new, y_p_new, psi_dot*dt, a*dt, 0, psi_dot - x[5]])
    return x_new

if __name__ == '__main__':
    main()
