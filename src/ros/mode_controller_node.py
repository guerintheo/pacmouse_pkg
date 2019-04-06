#!/usr/bin/python
import rospy
import signal
import numpy as np
from pidrone_pkg.msg import Mode, Battery, State


class ModeController(object):
    ''' An object used to control the mode '''

    def __init__(self):

        # Desired, current, and previous modes
        self.desired_mode = 'IDLE'
        self.curr_mode = 'IDLE'
        self.prev_mode = 'IDLE'

    # ROS Callback Methods
    ######################
    def force_mode_callback(self, msg):
        """FORCE Update the mode """
        self.prev_mode = self.curr_mode
        self.curr_mode = msg.mode
        if self.prev_mode != self.curr_mode:
            print self.curr_mode

    def desired_mode_callback(self, msg):
        """Update the current mode of the drone"""
        self.desired_mode = msg.mode

    def ctrl_c_handler(self, signal, frame):
        """Disarms and exits the program if ctrl-c is pressed"""
        print "\nCaught ctrl-c! About to Disarm!"
        self.cmd_mode_pub.publish('DISARMED')
        sys.exit()


def main():
    # ROS Setup
    ###########
    rospy.init_node("mode_controller")

    # Instantiate a ModeController object
    mc = ModeController()

    # Publishers
    ############
    mc.mode_pub = rospy.Publisher('/pacmouse/mode', Mode, queue_size=1)

    # Subscribers
    #############
    rospy.Subscriber("/pacmouse/buttons/mode", Mode, mc.mode_callback)
    
    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, mc.ctrl_c_handler)

    print 'Controlling Mode'
    r = rospy.Rate(30) # 100hz
    while not rospy.is_shutdown():
        try:

            # Finite State Machine
            ######################
            if mc.curr_mode == 'RUNNING':
                if mc.desired_mode == 'PAUSED':
                    mc.cmd_mode_pub.publish('PAUSED')
                elif mc.desired_mode == 'RUNNING':
                    print 'sending run command'
                    mc.cmd_mode_pub.publish('RUNNING')
                    rospy.sleep(1)
                else:
                    print 'Cannot transition from Mode %s to Mode %s' % (mc.curr_mode, mc.desired_mode)

            elif mc.curr_mode == 'PAUSED':
                if mc.desired_mode == 'PAUSED':
                    mc.cmd_mode_pub.publish('PAUSED')
                elif mc.desired_mode == 'RUNNING':
                    print 'sending run command'
                    mc.cmd_mode_pub.publish('RUNNING')
                elif mc.desired_mode == 'IDLE':
                    print 'sending idle command'
                    mc.cmd_mode_pub.publish('IDLE')
                else:
                    print 'Cannot transition from Mode %s to Mode %s' % (mc.curr_mode, mc.desired_mode)

            elif mc.curr_mode == 'IDLE':
                if mc.desired_mode == 'IDLE':
                    mc.cmd_mode_pub.publish('IDLE')
                elif mc.desired_mode == 'PAUSED':
                    print 'sending pause command'
                    mc.cmd_mode_pub.publish('PAUSED')
                elif mc.desired_mode == 'RUNNING':
                    print 'sending run command'
                    mc.cmd_mode_pub.publish('RUNNING')
                else:
                    print 'Cannot transition from Mode %s to Mode %s' % (mc.curr_mode, mc.desired_mode)

        except:
                print 'there was an internal error'
                print 'cannot transition to', mc.desired_mode
                sys.exit()
        r.sleep()

    mc.cmd_mode_pub.publish('IDLE')
    print 'Shutdown Received'
    print 'Sending DISARM command'


if __name__ == '__main__':
    main()