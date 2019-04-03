#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3  # TODO: Use a better message type
from pacmouse_pkg.src.hardware_interfaces.motors import Motors
from pacmouse_pkg.src.estimation_control.control import PID
import pacmouse_pkg.src.params as p

class MotorControlNode:
    
    def __init__(self):
        rospy.init_node('motor_control_node')
        rospy.Subscriber('/pacmouse/motors/cmd', Vector3, self.cmd_callback)
        rospy.Subscriber('/pacmouse/encoders', Vector3, self.encoders_callback)
        self.pwm_publisher = rospy.Publisher('/pacmouse/motors/pwm', Vector3, queue_size=1)
        
        self.m = Motors()
        self.sp = np.zeros(2)
        self.encoders = np.zeros(2)

        self.spin()
        
    def cmd_callback(self, data):
        """
        Set the motor speeds based on the input command.
        """
        left = data.x
        right = data.y
        print('Got command: left={}, right={}'.format(left, right))
        self.sp = np.array([left, right])

    def encoders_callback(self, data):
        """
        Read from the encoders
        """
        left = data.x
        right = data.y
        self.encoders = np.array([left, right])
        
    def spin(self):
        pid_L = PID(*p.motor_controller_pid, control_range=[-1,1])
        pid_R = PID(*p.motor_controller_pid, control_range=[-1,1])

        r = rospy.Rate(p.motor_control_freq)
        pwm_msg = Vector3()

        while not rospy.is_shutdown():

            err_L, err_R = self.sp - self.encoders # find the error in the motor velocity

            control_coeff = 1.
            cmd_L = pid_L.step(err_L, 1./p.motor_control_freq) + self.sp[0] * control_coeff
            cmd_R = pid_R.step(err_R, 1./p.motor_control_freq) + self.sp[1] * control_coeff

            self.m.set(cmd_L, cmd_R)

            pwm_msg.x = cmd_L
            pwm_msg.y = cmd_R
            self.pwm_publisher.publish(pwm_msg)

            r.sleep()

        self.m.stop()

        
if __name__ == '__main__':
    mc = MotorControlNode()