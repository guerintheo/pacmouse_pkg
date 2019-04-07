#!/usr/bin/python
import rospy
import signal
import sys

from std_msgs.msg import Bool, Empty, String, Int16
from pacmouse_pkg.msg import LED  # TODO: Create this custom message


class Mode(Enum):
    IDLE = 0
    EXPLORING = 1
    SHORTEST_PATH_SOLVING = 2
    RETURNING = 3  # returning to start cell of maze
    
    # Sub-modes of IDLE:
    SET_MODE = 4
    SET_SPEED = 5
    SET_MAZE_REVERT = 6
    RESTART = 7


class ModeController(object):
    """
    Class that controls and keeps track of the mode/state of the robot. This
    class acts as a state machine.
    """

    def __init__(self):
        # Previous, current, and desired top-level modes
        self.prev_mode = None
        self.curr_mode = Mode.IDLE
        self.desired_mode = None
        # TODO (Theo): Does it make sense to use desired_mode with the
        # paradigm we have set up here? Desired mode could be a sort of
        # "mode" message sent be a node trying to change the mode of the
        # mode controller, I feel like. There may be some other
        # intuitive use for such a variable, but not sure currently, so
        # probably will not use it. Consider changing usage.
        
        self.idle_submode = None
        
        # Boolean states
        self.button1_state = None
        self.button2_state = None
        self.button3_state = None
        self.button4_state = None
        
        self.rotary_option_threshold = 100  # number of ticks # TODO: Set to an intuitive value
        
        signal.signal(signal.SIGINT, self.sigint_handler)
        
        self.init_publishers()
        self.init_subscribers()
        
    def init_publishers(self):
        """Initialize ROS publishers."""
        self.set_motor_arm_pub = rospy.Publisher('/pacmouse/mode/set_motor_arm',
                                                 Bool,
                                                 queue_size=1)
        self.set_leds_pub = rospy.Publisher('/pacmouse/mode/set_leds', LED,
                                            queue_size=1)
        self.zero_pose_pub = rospy.Publisher('/pacmouse/mode/zero_pose', Empty,
                                             queue_size=1)
        self.zero_heading_pub = rospy.Publisher('/pacmouse/mode/zero_heading',
                                                Empty,
                                                queue_size=1)
        self.load_maze_pub = rospy.Publisher('/pacmouse/mode/load_maze', String,
                                             queue_size=1)
        self.set_plan_mode_pub = rospy.Publisher('/pacmouse/mode/set_plan_mode',
                                                 String,
                                                 queue_size=1)
        self.set_plan_speed_pub = rospy.Publisher(
                                    '/pacmouse/mode/set_plan_speed', String,
                                    queue_size=1)
        self.set_encoder_dial_pub = rospy.Publisher(
                                        '/pacmouse/mode/set_encoder_dial', Bool,
                                        queue_size=1)
    
    def init_subscribers(self):
        """Initialize ROS subscribers."""
        # Button subscribers
        rospy.Subscriber('/pacmouse/buttons/1', Bool, self.cb_button1)
        rospy.Subscriber('/pacmouse/buttons/2', Bool, self.cb_button2)
        rospy.Subscriber('/pacmouse/buttons/3', Bool, self.cb_button3)
        rospy.Subscriber('/pacmouse/buttons/4', Bool, self.cb_button4)
        
        # Rotary dial subscriber
        rospy.Subsciber('/pacmouse/encoder_dial', Int16, self.cb_encoder_dial)
        
    def cb_button1(self, data):
        """
        Callback for a button1 toggle event.
        """
        self.button1_state = data.data
        if self.curr_mode is not Mode.IDLE:
            return
        # Only act on button toggle when in IDLE mode.
        if self.button1_state and self.idle_submode is None:
            # Button toggled high while not in a sub-mode of the IDLE mode
            # (i.e., we are in the top-level IDLE mode): enter SET_MODE state in
            # the IDLE state machine. In this mode, we now accept rotary encoder
            # input or another toggle of button1.
            self.idle_submode = Mode.SET_MODE
            self.rotary_dial_value = 0
            # Signal to the encoders that we wish to receive encoder ticks now.
            # We do this setting dynamically so as to avoid overhead of constant
            # encoder callbacks in the mode controller when not IDLE (e.g., when
            # driving)
            self.set_encoder_dial_pub.publish(True)
            # TODO
        if not self.button1_state and self.idle_submode is Mode.SET_MODE:
            # Button toggled low while in SET_MODE mode: depending on the value
            # of the rotary option, either go back to top-level IDLE mode, go
            # to EXPLORING mode, or go to SHORTEST_PATH_SOLVING mode.
            self.set_encoder_dial_pub.publish(False)
            if self.rotary_dial_value >= self.rotary_option_threshold:
                self.desired_mode = Mode.SHORTEST_PATH_SOLVING
            elif self.rotary_dial_value <= -self.rotary_option_threshold:
                self.desired_mode = Mode.EXPLORING
            else:
                # Remain IDLE
                self.curr_mode = Mode.IDLE
        
    def cb_button2(self, data):
        """
        Callback for a button2 toggle event.
        """
        self.button2_state = data.data
        
    def cb_button3(self, data):
        """
        Callback for a button3 toggle event.
        """
        self.button3_state = data.data
        
    def cb_button4(self, data):
        """
        Callback for a button4 toggle event.
        """
        self.button4_state = data.data
        
    def encoder_dial(self, data):
        """
        Callback for the encoder dial. This callback increments or decrements a
        dial value (depending on direction of rotation of the encoder tick),
        which allows other mode events to associate rotary ticks with an option,
        depending on which sub-mode of the IDLE mode we are in.
        
        This callback also publishes to the LED node to indicate rotary option
        selection progress.
        """
        self.rotary_dial_value += data.data
        # TODO: LEDs
        
        
    def sigint_handler(self, signal, frame):
        """
        Exit cleanly upon receiving a SIGINT signal (e.g., from Ctrl-C keyboard
        interrupt).
        """
        # TODO
        # E.g., send message to disarm motors
        sys.exit()


def main():
    rospy.init_node('mode_controller')
    mc = ModeController()
    print('Mode controller node started.')
    rospy.spin()

if __name__ == '__main__':
    main()