#!/usr/bin/python
import rospy
import signal
import numpy as np
from pidrone_pkg.msg import Mode, Battery, State


class ModeController(object):
    ''' An object used to control the mode '''

    def __init__(self):

        # Desired, current, and previous modes of the drone
        self.desired_mode = 'IDLE'
        self.curr_mode = 'IDLE'
        self.prev_mode = 'IDLE'

        # Publisher to send the commanded mode to
        self.cmd_mode_pub = None

    # ROS Callback Methods
    ######################
    def mode_callback(self, msg):
        """Update the current mode of the drone"""
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
    rospy.Subscriber("/pacmouse/maze_status", Mode, mc.mode_callback)
    

    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, mc.ctrl_c_handler)

    print 'Controlling Mode'
    r = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        try:
            # if the current or desired mode is anything other than disarmed
            # preform as safety check
            if mc.curr_mode != 'DISARMED' or mc.desired_mode != 'DISARMED':
                # Break the loop if a safety check has failed
                if mc.shouldIDisarm():
                    break

            # Finite State Machine
            ######################
            if mc.curr_mode == 'DISARMED':
                if mc.desired_mode == 'DISARMED':
                    mc.cmd_mode_pub.publish('DISARMED')
                elif mc.desired_mode == 'ARMED':
                    print 'sending arm command'
                    mc.cmd_mode_pub.publish('ARMED')
                    rospy.sleep(1)
                else:
                    print 'Cannot transition from Mode %s to Mode %s' % (mc.curr_mode, mc.desired_mode)

            elif mc.curr_mode == 'ARMED':
                if mc.desired_mode == 'ARMED':
                    mc.cmd_mode_pub.publish('ARMED')
                elif mc.desired_mode == 'FLYING':
                    print 'sending fly command'
                    mc.cmd_mode_pub.publish('FLYING')
                elif mc.desired_mode == 'DISARMED':
                    print 'sending disarm command'
                    mc.cmd_mode_pub.publish('DISARMED')
                else:
                    print 'Cannot transition from Mode %s to Mode %s' % (mc.curr_mode, mc.desired_mode)

            elif mc.curr_mode == 'FLYING':
                if mc.desired_mode == 'FLYING':
                    mc.cmd_mode_pub.publish('FLYING')
                elif mc.desired_mode == 'DISARMED':
                    print 'sending disarm command'
                    mc.cmd_mode_pub.publish('DISARMED')
                else:
                    print 'Cannot transition from Mode %s to Mode %s' % (mc.curr_mode, mc.desired_mode)

        except:
                print 'there was an internal error'
                print 'cannot transition to', mc.desired_mode
                sys.exit()
        r.sleep()

    mc.cmd_mode_pub.publish('DISARMED')
    print 'Shutdown Received'
    print 'Sending DISARM command'


if __name__ == '__main__':
    main()