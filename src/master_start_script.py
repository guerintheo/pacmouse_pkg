#!/usr/bin/python
# import sys, os
# if os.geteuid() != 0:
#     exit("YOU GOTTA RUN ME WITH SUDO BRO!!!")

import sys
import rospy
from std_msgs.msg import Empty
from subprocess import Popen
import time
import roslaunch


class MasterStartNode(object):
    """
    This node is responsible for starting up the ROS nodes for competition
    setup and for handling restarts of the software stack. The intention is for
    it to be run by its own launch file, which will start up roscore at the same
    time. Then, this node launches the main competition-day nodes via a launch
    file. From that point on, this node just waits to receive a message from the
    mode controller indicating a desired restart, at which point this node
    kills the roslaunch process that it spawned and attempts to restart it. Note
    that roscore stays up the whole time during this process, as this node
    is relying on ROS for message passing. This node should be run on the AP
    Master (pacmouse_ap, the Pi 3), as it will run roscore for the Pi Zero to
    connect to.
    """

    def __init__(self):
        print('Starting up the master_start_node.')
        rospy.init_node('master_start_node')
        rospy.on_shutdown(self.shutdown)

        restart_topic = '/pacmouse/restart'
        print('Subscribing to the restart topic at {}.'.format(restart_topic))
        rospy.Subscriber(restart_topic, Empty, self.cb_restart)

        # TODO: May need full path
        self.path = 'launch/gameday/gameday_launch_both.launch'
        if len(sys.argv) == 2:
            if sys.argv[1] == 'test':
                self.path = 'launch/testing.launch'

        self.launch_new_process()

        print('Waiting for a restart message or a shutdown.')
        rospy.spin()

    def launch_new_process(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.path])
        print('Launching launch file at: {}'.format(self.path))
        self.launch.start()

    def shutdown(self):
        print('Terminating launched launch file.')
        self.launch.shutdown()
        print('Terminating this process.')

    def cb_restart(self, data):
        """
        Received a message requesting that we shut down and restart the software
        stack.
        """
        print('Terminating launched launch file.')
        self.launch.shutdown()
        self.launch_new_process()

if __name__ == '__main__':
    master_start = MasterStartNode()
