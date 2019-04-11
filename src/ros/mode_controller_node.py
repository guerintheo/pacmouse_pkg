#!/usr/bin/python
import rospy
import signal
import sys
import time
from enum import Enum

from std_msgs.msg import Bool, Empty, String, Int16, Float64
from pacmouse_pkg.msg import LED, Drive


class Mode(Enum):
    IDLE = 0
    EXPLORING = 1
    SHORTEST_PATH_SOLVING = 2
    RETURNING = 3  # returning to start cell of maze

    # Sub-modes of IDLE:
    SET_MODE = 4
    SET_SPEED = 5
    SET_MAZE_REVERT = 6
    SET_RESTART = 7


class ModeController(object):
    """
    Class that controls and keeps track of the mode/state of the robot. This
    class acts as a state machine.
    """

    def __init__(self):


        self.idle_submode = None

        self.rotary_option_threshold = 100  # number of ticks # TODO: Set to an intuitive value
        # Number of seconds to wait before starting shortest-path solving or
        # exploration if toggled by a human
        self.zero_pose_and_heading_delay_seconds = 3

        # TODO: Get this information from planner or pose estimator, or
        # whichever node determines that we have reached maze goal
        self.found_path_to_maze_goal = False

        signal.signal(signal.SIGINT, self.sigint_handler)

        self.init_publishers()
        self.init_subscribers()

        self.led_modes = ModeLEDSignalFunctions(self)
        # Previous and current top-level modes
        self.prev_mode = None
        self.set_curr_mode_idle()

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
                                    '/pacmouse/mode/set_plan_speed', Float64,
                                    queue_size=1)
        self.restart_software_pub = rospy.Publisher('/pacmouse/restart', Empty,
                                                    queue_size=1)

    def init_subscribers(self):
        """Initialize ROS subscribers."""
        # Button subscribers
        rospy.Subscriber('/pacmouse/buttons/1', Bool, self.cb_button1)
        rospy.Subscriber('/pacmouse/buttons/2', Bool, self.cb_button2)
        rospy.Subscriber('/pacmouse/buttons/3', Bool, self.cb_button3)
        rospy.Subscriber('/pacmouse/buttons/4', Bool, self.cb_button4)

        # Rotary dial subscriber
        rospy.Subscriber('/pacmouse/encoders/position', Drive, self.cb_encoder_dial)

    def set_curr_mode_idle(self):
        self.curr_mode = Mode.IDLE
        self.led_function = self.led_modes.set_leds_for_idle

    def cb_button1(self, data):
        """
        Callback for a button1 toggle event.

        This button is used to enter or exit the SET_MODE sub-mode of IDLE if we
        are currently in the IDLE mode.
        """
        button1_state = data.data
        # Only act on button toggle when in IDLE mode.
        if self.curr_mode is not Mode.IDLE:
            return
        if button1_state and self.idle_submode is None:
            # Button toggled high while not in a sub-mode of the IDLE mode
            # (i.e., we are in the top-level IDLE mode): enter SET_MODE state in
            # the IDLE state machine. In this mode, we now accept rotary encoder
            # input or another toggle of button1.
            self.idle_submode = Mode.SET_MODE
            self.rotary_dial_start_value = self.rotary_dial_value
            self.led_function = self.led_modes.set_leds_for_set_mode
            self.led_function()
            # Disarm the motors while in IDLE mode
            self.set_motor_arm_pub.publish(False)
        elif not button1_state and self.idle_submode is Mode.SET_MODE:
            # Button toggled low while in SET_MODE mode: depending on the value
            # of the rotary option, either go back to top-level IDLE mode, go
            # to EXPLORING mode, or go to SHORTEST_PATH_SOLVING mode.
            self.idle_submode = None  # we are exiting an IDLE sub-mode
            rotary_dial_angle_diff = self.rotary_dial_value - self.rotary_dial_start_value
            if rotary_dial_angle_diff >= self.rotary_option_threshold:
                if self.found_path_to_maze_goal:
                    # Can only do shortest-path solve if we have found the
                    # goal/center of the maze
                    self.start_shortest_path_solve(toggled_by_human=True)
                else:
                    self.set_curr_mode_idle()
            elif rotary_dial_angle_diff <= -self.rotary_option_threshold:
                self.start_exploring(toggled_by_human=True)
            else:
                # Remain IDLE
                self.set_curr_mode_idle()

    def cb_button2(self, data):
        """
        Callback for a button2 toggle event.

        This button is used to enter or exit the SET_SPEED sub-mode of IDLE if
        we are currently in the IDLE mode.
        """
        button2_state = data.data
        # Only act on button toggle when in IDLE mode.

        if self.curr_mode is not Mode.IDLE:
            return
        if button2_state and self.idle_submode is None:
            # Button toggled high while not in a sub-mode of the IDLE mode
            # (i.e., we are in the top-level IDLE mode): enter SET_SPEED state
            # in the IDLE state machine. In this mode, we now accept rotary
            # encoder input or another toggle of button2.
            print 'button2 toggled high'
            self.idle_submode = Mode.SET_SPEED
            self.rotary_dial_start_value = self.rotary_dial_value
            self.led_function = self.led_modes.set_leds_for_set_speed
            self.led_function()
            # Disarm the motors while in IDLE mode
            self.set_motor_arm_pub.publish(False)
        elif not button2_state and self.idle_submode is Mode.SET_SPEED:
            # Button toggled low while in SET_SPEED mode
            self.idle_submode = None  # we are exiting an IDLE sub-mode
            rotary_dial_angle_diff = self.rotary_dial_value - self.rotary_dial_start_value
            self.set_plan_speed_pub.publish(rotary_dial_angle_diff)
            self.set_curr_mode_idle()

    def cb_button3(self, data):
        """
        Callback for a button3 toggle event.

        This button is used to enter or exit the SET_MAZE_REVERT sub-mode of
        IDLE if we are currently in the IDLE mode.
        """
        button3_state = data.data
        # Only act on button toggle when in IDLE mode.
        if self.curr_mode is not Mode.IDLE:
            return
        if button3_state and self.idle_submode is None:
            # Button toggled high while not in a sub-mode of the IDLE mode
            # (i.e., we are in the top-level IDLE mode): enter SET_MAZE_REVERT
            # state in the IDLE state machine. In this mode, we now accept
            # rotary encoder input or another toggle of button3.
            self.idle_submode = Mode.SET_MAZE_REVERT
            self.rotary_dial_start_value = self.rotary_dial_value
            self.led_function = self.led_modes.set_leds_for_maze_revert
            self.led_function()
            # Disarm the motors while in IDLE mode
            self.set_motor_arm_pub.publish(False)
        elif not button3_state and self.idle_submode is Mode.SET_MAZE_REVERT:
            # Button toggled low while in SET_MAZE_REVERT mode
            self.idle_submode = None  # we are exiting an IDLE sub-mode
            # TODO: Pass in a file name of a maze to load into memory based on the rotary encoder value indicating how far back in time to revert back to. Coordinate this with the LED indicators.
            rotary_dial_angle_diff = self.rotary_dial_value - self.rotary_dial_start_value
            # self.load_maze_pub.publish()
            self.set_curr_mode_idle()

    def cb_button4(self, data):
        """
        Callback for a button4 toggle event.

        This button is used to enter or exit the SET_RESTART sub-mode of IDLE if
        we are currently in the IDLE mode.
        """
        button4_state = data.data
        # Only act on button toggle when in IDLE mode.
        if self.curr_mode is not Mode.IDLE:
            return
        if button4_state and self.idle_submode is None:
            # Button toggled high while not in a sub-mode of the IDLE mode
            # (i.e., we are in the top-level IDLE mode): enter the SET_RESTART
            # state in the IDLE state machine. In this mode, we now accept
            # rotary encoder input or another toggle of button4.
            self.idle_submode = Mode.SET_RESTART
            self.rotary_dial_start_value = self.rotary_dial_value
            self.led_function = self.led_modes.set_leds_for_restart
            self.led_function()
            # Disarm the motors while in IDLE mode
            self.set_motor_arm_pub.publish(False)
        elif not button4_state and self.idle_submode is Mode.SET_RESTART:
            # Button toggled low while in SET_RESTART mode
            self.idle_submode = None  # we are exiting an IDLE sub-mode
            # Based on the rotary encoder ticks, determine whether or not to
            # restart the software stack
            rotary_dial_angle_diff = self.rotary_dial_value - self.rotary_dial_start_value
            if rotary_dial_angle_diff >= self.rotary_option_threshold:
                self.restart_software_stack()
            else:
                # Remain IDLE
                self.set_curr_mode_idle()

    def cb_encoder_dial(self, data):
        """
        Callback for the encoder dial. This callback increments or decrements a
        dial value (depending on direction of rotation of the encoder tick),
        which allows other mode events to associate rotary ticks with an option,
        depending on which sub-mode of the IDLE mode we are in.

        This callback also publishes to the LED node to indicate rotary option
        selection progress.
        """
        self.rotary_dial_value = data.R
        # Only set LEDs on encoder callback when not in IDLE mode
        print 'idle_submode: {}'.format(self.idle_submode)
        if self.idle_submode is None:
            return
        self.led_function()

    def restart_software_stack(self):
        """
        Restart the software stack.

        This would typically only be called if there is an error that an
        operator has deemed requires a hard restart of the software stack, such
        as diverging pose or maze estimates.
        """
        # Disarm the motors before trying to restart
        self.set_motor_arm_pub.publish(False)
        time.sleep(1)
        self.restart_software_pub.publish(Empty())

    def start_shortest_path_solve(self, toggled_by_human=False):
        """
        Set up the robot to transition into shortest-path solve mode.

        Summary of what is done in this function:
            - Arm the motors
            - Send message to planner to plan a shortest path
            - Send message to maze estimator to fix the maze estimate
              (Could perhaps just have planner fix its maze representation.
              TODO: Figure out a good way to do this.)
            - Send message to LEDs to have them reflect shortest-path solving
              mode
            - If toggled by human, then also send message to pose estimator and
              IMU to zero the pose and heading values, respectively
        """
        self.curr_mode = Mode.SHORTEST_PATH_SOLVING
        if toggled_by_human:
            print('Sleeping {} seconds before starting shortest path.'.format(
                  self.zero_pose_and_heading_delay_seconds))
            time.sleep(self.zero_pose_and_heading_delay_seconds)
            print('Zeroing pose and heading.')
            self.zero_pose_pub.publish(Empty())
            self.zero_heading_pub.publish(Empty())
        # Arm the motors
        self.set_motor_arm_pub.publish(True)
        self.set_plan_mode_pub.publish('SHORTEST_PATH_SOLVING')
        self.led_function = self.led_modes.set_leds_for_shortest_path
        self.led_function()  # TODO: Figure out if we want LEDs to change during a shortest path run

    def start_exploring(self, toggled_by_human=False):
        """
        Set up the robot to transition into exploring mode.

        Summary of what is done in this function:
            - Arm the motors
            - Send message to planner to plan for maze exploration
            - Send message to maze estimator to activate maze estimation (TODO)
            - Send message to LEDs to have them reflect exploration mode
            - If toggled by human, then also send message to pose estimator and
              IMU to zero the pose and heading values, respectively
        """
        self.curr_mode = Mode.EXPLORING
        if toggled_by_human:
            print('Sleeping {} seconds before starting exploration.'.format(
                  self.zero_pose_and_heading_delay_seconds))
            time.sleep(self.zero_pose_and_heading_delay_seconds)
            print('Zeroing pose and heading.')
            self.zero_pose_pub.publish(Empty())
            self.zero_heading_pub.publish(Empty())
        # Arm the motors
        self.set_motor_arm_pub.publish(True)
        self.set_plan_mode_pub.publish('EXPLORING')
        self.led_function = self.led_modes.set_leds_for_exploring
        self.led_function()  # TODO: Figure out if we want LEDs to change during exploration, perhaps to indicate confidence in pose and/or maze estimate, potential estimate divergence, etc.

    def sigint_handler(self, signal, frame):
        """
        Exit cleanly upon receiving a SIGINT signal (e.g., from Ctrl-C keyboard
        interrupt).
        """
        # TODO
        # Send message to disarm motors
        self.set_motor_arm_pub.publish(False)
        sys.exit()

class ModeLEDSignalFunctions(object):

    GREEN = '0x00FF00'
    OFF = '0x000000'
    ORANGE = '0xFF7F00'
    RED = '0xFF0000'

    def __init__(self, mode_controller):
        self.mc = mode_controller
        self.set_leds_pub = self.mc.set_leds_pub

    def clear_leds(self):
        for i in range(3):
            # Clear each of the 3 LEDs
            self.clear_led(i)

    def clear_led(self, led_num):
        led_msg = LED()
        led_msg.hex_color = OFF
        led_msg.led_num = led_num
        self.set_leds_pub.publish(led_msg)

    def set_leds_for_idle(self):
        """Set LED 0 to green."""
        print 'In Mode Idle'
        led_msg = LED()
        led_msg.led_num = 0
        led_msg.hex_color = GREEN
        self.set_leds_pub.publish(led_msg)
        # Clear the other two LEDs
        self.clear_led(1)
        self.clear_led(2)

    def set_leds_for_set_mode(self):
        print('In mode SET_MODE')
        led_msg = LED()
        led_msg.led_num = 1
        led_msg.hex_color = ORANGE
        self.set_leds_pub.publish(led_msg)

        rotary_dial_angle_diff = self.mc.rotary_dial_value - self.mc.rotary_dial_start_value
        print('Rotary angle diff: {}'.format(rotary_dial_angle_diff))
        if rotary_dial_angle_diff >= self.mc.rotary_option_threshold:
            led_msg.led_num = 2
            if self.mc.found_path_to_maze_goal:
                # Can only do shortest-path solve if we have found the
                # goal/center of the maze. Indicate this with LED 2 set to green
                led_msg.hex_color = GREEN
            else:
                led_msg.hex_color = RED
            self.set_leds_pub.publish(led_msg)
            self.clear_led(0)
        elif rotary_dial_angle_diff <= -self.rotary_option_threshold:
            # Exploration mode. Set LED 0 to green
            led_msg.led_num = 0
            led_msg.hex_color = GREEN
            self.set_leds_pub.publish(led_msg)
            self.clear_led(2)
        else:
            # Remain IDLE
            self.clear_led(0)
            self.clear_led(2)

    def set_leds_for_set_speed(self):
        print('In mode SET_SPEED')
        self.set_leds_spectrum()

    def set_leds_spectrum(self):
        led_msg = LED()
        rotary_dial_angle_diff = self.mc.rotary_dial_value - self.mc.rotary_dial_start_value
        print('Rotary angle diff: {}'.format(rotary_dial_angle_diff))

        led_msg.led_num = 0
        led_msg.hex_color = self.compute_color_on_spectrum(rotary_dial_angle_diff)
        self.set_leds_pub.publish(led_msg)

        led_msg.led_num = 1
        if rotary_dial_angle_diff >= self.mc.rotary_option_threshold:
            rotary_dial_angle_diff = rotary_dial_angle_diff - self.mc.rotary_option_threshold
            led_msg.hex_color = self.compute_color_on_spectrum(rotary_dial_angle_diff)
        else:
            led_msg.hex_color = OFF
        self.set_leds_pub.publish(led_msg)

        led_msg.led_num = 2
        if rotary_dial_angle_diff >= self.mc.rotary_option_threshold:
            rotary_dial_angle_diff = rotary_dial_angle_diff - self.mc.rotary_option_threshold
            led_msg.hex_color = self.compute_color_on_spectrum(rotary_dial_angle_diff)
        else:
            led_msg.hex_color = OFF
        self.set_leds_pub.publish(led_msg)

    def compute_color_on_spectrum(self, angle):
        """Map an angle value to a color between red and blue."""
        ratio = angle/self.mc.rotary_option_threshold
        r = g = b = 0
        inner_ratio = self._calculate_inner_ratio(ratio)
        if ratio < 1.0/6.0:
            r = 255
            g = 255*inner_ratio
            b = 0
        elif ratio < 1.0/3.0:
            r = 255 - 255*inner_ratio
            g = 255
            b = 0
        elif ratio < 1.0/2.0:
            r = 0
            g = 255
            b = 255*inner_ratio
        elif ratio < 2.0/3.0:
            r = 0
            g = 255 - 255*inner_ratio
            b = 255
        elif ratio < 5.0/6.0:
            r = 255*inner_ratio
            g = 0
            b = 255
        else:
            r = 255
            g = 0
            b = 255 - 255*inner_ratio
        return self._rgb_to_hex_str(r, g, b)

    def _calculate_inner_ratio(self, num):
        return (num % (1.0/6.0))/(1.0/6.0)

    def _rgb_to_hex_str(self, r, g, b):
        return hex((r << 16) + (g << 8) + b)

    def set_leds_for_maze_revert(self):
        print('In mode SET_MAZE_REVERT')
        self.set_leds_spectrum()

    def set_leds_for_restart(self):
        # TODO
        pass

    def set_leds_for_shortest_path(self):
        # TODO
        pass

    def set_leds_for_exploring(self):
        # TODO
        pass

def main():
    rospy.init_node('mode_controller')
    mc = ModeController()
    print('Mode controller node started.')
    rospy.spin()

if __name__ == '__main__':
    main()
