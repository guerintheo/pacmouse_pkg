#!/usr/bin/python
import rospy
import signal
import numpy as np
import enums
from msg import AgentState, LightState
from std_msgs.msg import Bool


class ModeController(object):
    ''' An object used to control the mode '''

    def __init__(self):

        # current, and previous modes of the game
        self.curr_mode = enums.PAUSED
        self.prev_mode = enums.PAUSED

        # Publisher to send the commanded mode to
        self.mode_pub = None

    # ROS Callback Methods
    ######################
    def light_state_callback(self, msg):
        """ store the game mode """
        self.prev_mode = self.mode
        self.curr_mode = msg.mode
        # notify user of mode change
        if self.prev_mode != self.curr_mode:
            print self.curr_mode

    def button1_callback(self, msg):
        if msg.data:
            # do something when button is pressed
            pass

    def button2_callback(self, msg):
        if msg.data:
            # do something when button is pressed
            pass

    def button3_callback(self, msg):
        if msg.data:
            # do something when button is pressed
            pass

    def button4_callback(self, msg):
        if msg.data:
            # do something when button is pressed
            pass


    def ctrl_c_handler(self, signal, frame):
        """sets mode to PAUSED and exits the program if ctrl-c is pressed"""
        print "\nCaught ctrl-c. Pausing and Exiting"
        self.mode_pub.publish(enums.PAUSED)
        sys.exit()


def main():
    # ROS Setup
    ###########
    rospy.init_node("mode_controller")

    # Instantiate a ModeController object
    mc = ModeController()

    # Publishers
    ############
    mc.mode_pub = rospy.Publisher('/pacmouse/mode', int8, queue_size=1, latch=True)

    # Subscribers
    #############

    rospy.Subscriber("/pacmouse/light_state", LightState, mc.light_state_callback)
    #Button Subscribers:
    rospy.Subscriber("/pacmouse/buttons/1", Bool, mc.button1_callback)
    rospy.Subscriber("/pacmouse/buttons/2", Bool, mc.button2_callback)
    rospy.Subscriber("/pacmouse/buttons/3", Bool, mc.button3_callback)
    rospy.Subscriber("/pacmouse/buttons/4", Bool, mc.button4_callback)


    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, mc.ctrl_c_handler)

    print 'Controlling Mode'
    # TODO: choose publishing rate
    r = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        # TODO: any looping needed?
        r.sleep()

    mc.mode_pub.publish(enums.PAUSED)
    print 'Shutdown Received'
    print 'PAUSED'


if __name__ == '__main__':
    main()
