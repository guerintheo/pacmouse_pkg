#!/usr/bin/python
import rospy
import signal
import sys

from std_msgs.msg import Bool, Empty, String
from pacmouse_pkg.msg import LED  # TODO: Create this custom message


class Mode(Enum):
    IDLE = 0
    EXPLORING = 1
    SHORTEST_PATH_SOLVING = 2
    RETURNING = 3  # returning to start cell of maze


class ModeController(object):
    """
    Class that controls the mode
    """

    def __init__(self):
        # Previous, current, and desired modes
        self.prev_mode = None
        self.curr_mode = Mode.IDLE
        self.desired_mode = None
        
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
    
    def init_subscribers(self):
        """Initialize ROS subscribers."""
        rospy.Subscriber('/pacmouse/buttons/1', Bool, self.cb_button1)
        rospy.Subscriber('/pacmouse/buttons/2', Bool, self.cb_button2)
        rospy.Subscriber('/pacmouse/buttons/3', Bool, self.cb_button3)
        rospy.Subscriber('/pacmouse/buttons/4', Bool, self.cb_button4)

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