#!/usr/bin/python
import rospy
from geometry_msgs.msg import Vector3  # TODO: Use a better message type
from pacmouse_pkg.src.hardware_interfaces.motors import Motors


class MotorControlNode:
    
    def __init__(self):
        rospy.init_node('motor_control_node')
        rospy.Subscriber('/pacmouse/cmd_motors', Vector3, self.cmd_callback)
        self.m = Motors()
        self.listen_for_commands()
        
    def cmd_callback(self, data):
        """
        Set the motor speeds based on the input command.
        """
        left = data.x
        right = data.y
        print('Got command: left={}, right={}'.format(left, right))
        self.m.set(left, right)
        
    def listen_for_commands(self):
        rospy.spin()
        
if __name__ == '__main__':
    mc = MotorControlNode()