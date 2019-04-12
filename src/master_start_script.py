#!/usr/bin/python
import rospy
from std_msgs.msg import Empty
from subprocess import Popen
import time
import sys

# TODO: Handle Ctrl-C

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
    is relying on ROS for message passing.
    """

    def __init__(self):
        self.process_is_active = False
        print('Starting up the master_start_node.')
        rospy.init_node('master_start_node')
        restart_topic = '/pacmouse/restart'
        print('Subscribing to the restart topic at {}.'.format(restart_topic))
        rospy.Subscriber(restart_topic, Empty, self.cb_restart)
        self.process_str = 'roslaunch pacmouse_pkg gameday.launch'
        if len(sys.argv) == 2:
            if sys.argv[1] == 'test':
                self.process_str = 'roslaunch pacmouse_pkg testing.launch'
        self.launch_process()
        rospy.spin()

    def launch_process(self):
        print('Launching process: {}'.format(self.process_str))
        # NOTE: shell=True can be a security hazard if given
        #       unsanitized/untrusted input
        self.launched_process = Popen(self.process_str,
                                      shell=True)
        self.process_is_active = True

    def cb_restart(self, data):
        """
        Received a message requesting that we shut down the software stack.
        """
        if not self.process_is_active:
            print('Process is not currently launched.')
            return
        self.process_is_active = False
        # Send SIGTERM to roslaunch process
        self.launched_process.terminate()
        # Use the blocking communicate() call to wait for process to terminate
        self.launched_process.communicate()
        # TODO: Consider issuing a SIGKILL signal soon after terminate() but
        # before waiting on communicate(), in case SIGTERM is handled such that
        # the process does not terminate?
        time.sleep(3)
        self.launch_process()

if __name__ == '__main__':
    master_start = MasterStartNode()
