#!/usr/bin/python
import rospy
from geometry_msgs.msg import Vector3  # TODO: Use a better message type


class KeyboardMotorController:
    """
    A ROS node that publishes motor commands, which a motor controller takes as
    input.
    """
    
    def __init__(self):
        rospy.init_node('keyboard_motor_command')
        self.cmd_pub = rospy.Publisher('/pacmouse/cmd_motors', Vector3, queue_size=1)
        self.cmd = [0, 0]  # [left, right]
        self.turn_increment = 0.015
        self.forward_increment = 0.08
        
    def read_input_loop(self):
        # rate = rospy.Rate(30)  # 30 Hz
        # while not rospy.is_shutdown():
            # self.cmd_pub.publish(m)
            # rate.sleep()
        while True:
            cmd = raw_input('Enter command: ')
            if cmd == 'a':
                # Left turn with WASD
                self.cmd[0] -= self.turn_increment
                self.cmd[1] += self.turn_increment
            elif cmd == 'd':
                # Right turn with WASD
                self.cmd[0] += self.turn_increment
                self.cmd[1] -= self.turn_increment
            elif cmd == 'w':
                # Forward
                self.cmd[0] += self.forward_increment
                self.cmd[1] += self.forward_increment
            elif cmd == 's':
                # Backward
                self.cmd[0] -= self.forward_increment
                self.cmd[1] -= self.forward_increment
            elif cmd == 'q':
                # q is stop
                self.cmd = [0, 0]
                
            self.publish_cmd()
            
    def publish_cmd(self):
        m = Vector3()
        m.x = self.cmd[0]
        m.y = self.cmd[1]
        self.cmd_pub.publish(m)
            
    
if __name__ == '__main__':
    keyboard_motor_controller = KeyboardMotorController()
    keyboard_motor_controller.read_input_loop()