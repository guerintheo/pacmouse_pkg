#!/usr/bin/python
# planner_node.py -- Outputs a /traj command based on current /pose_est
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import pacmouse_pkg.src.estimation_control.control as control
from pacmouse_pkg.src.utils.maze_parser import parse_maze_file
from pacmouse_pkg.src.utils.maze import Maze

pose_est = np.zeros(6) # x,y,theta

def pose_est_callback(msg):
    global pose_est
    pose_est[0] = msg.pose.position.x
    pose_est[1] = msg.pose.position.y

    orientation_q = msg.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_,_,theta) = euler_from_quaternion(orientation_list)
    pose_est[2] = theta


def main():
    rospy.init_node('planner_node')
    pose_est_sub = rospy.Subscriber('/pose_est', PoseStamped, pose_est_callback)
    traj_pub = rospy.Publisher('/traj', PoseStamped, queue_size=1)

    # Load maze
    (adj_matrix, size_x, size_y) = parse_maze_file('../gazebo_worlds/testmaze.txt')
    
    m = Maze(size_x, size_y)
    m.adj_matrix = adj_matrix

    # DEFINE TARGET CELL
    TARGET_CELL = np.array([5,0])

    traj = PoseStamped()
    loop_rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # TODO: Instead of target pose, we should specify a target pose + velocity

        target_pose = control.get_sp(pose_est, m, TARGET_CELL)

        traj.pose.position.x = target_pose[0]
        traj.pose.position.y = target_pose[1]

        quat = quaternion_from_euler(0,0,target_pose[2])
        traj.pose.orientation.x = quat[0]
        traj.pose.orientation.y = quat[1]
        traj.pose.orientation.z = quat[2]
        traj.pose.orientation.w = quat[3]


        traj_pub.publish(traj)

        loop_rate.sleep()

if __name__ == '__main__':
    main()
