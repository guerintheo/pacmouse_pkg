#!/usr/bin/python
import rospy
import pigpio
from geometry_msgs.msg import Vector3  # TODO: Could use a better message type


class PiGPIOHandler:
    """
    Class that handles the sending of commands to the pigpio handler
    (pigpio.pi()). That way, we only have one pigpio handler connecting to the
    pigpio daemon, and we only import pigpio once. Having more than one handler
    and/or import of pigpio may be causing GPIO resource/library conflicts that
    cause our motors and encoders to just drop out spontaneously (albeit
    rarely).
    
    Refer to this GitHub for potentially useful information about pigpio
    connections: https://github.com/joan2937/pigpio/issues/256
    
    NOTE: This paradigm may turn out to have way too much overhead for the high
    frequency of sampling the GPIO pins that we want to achieve.
    """
    
    def __init__(self):
        rospy.init_node('pigpio_handler_node')
        
        # Initialize subscribers:
        rospy.Subscriber('/pacmouse/pigpio/set_mode', Vector3, self.cb_set_mode)
        rospy.Subscriber('/pacmouse/pigpio/callback', Vector3, self.cb_callback)
        rospy.Subscriber('/pacmouse/pigpio/get_current_tick', Vector3, self.cb_get_current_tick)
        
        self.pi = pigpio.pi()
        
    def cb_set_mode(self, data):
        """
        self.pi.set_mode() callback.
        """
        pass
        
    def cb_callback(self, data):
        """
        self.pi.callback() callback.
        """
        pass
    
    def cb_get_current_tick(self, data):
        """
        self.pi.get_current_tick() callback.
        """
        pass