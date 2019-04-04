import pigpio
import time
import rospy
from geometry_msgs.msg import Vector3  # TODO: Use a better message type
import pacmouse_pkg.src.params as p
import numpy as np
from multiprocessing import Process, Array
from collections import deque


class Encoders:
    def __init__(self):
        rospy.init_node('encoder_node')
        self.publisher = rospy.Publisher('/pacmouse/encoders', Vector3, queue_size=1)

        self.pi = pigpio.pi()
        self.counts = dict()
        self.most_recent_rising = dict()
        self.most_recent_falling = dict()
        self.left_direction_count = 0
        self.right_direction_count = 0
        self.alpha = 0.5
        
        # TODO: try changing this to pigpio
        for i, pin in enumerate(p.encoder_pins):
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.callback(pin, pigpio.EITHER_EDGE, self._callback)
            self.counts[pin] = 0
            self.most_recent_rising[pin] = time.time()
            self.most_recent_falling[pin] = time.time()

        self.spin()

    def _callback(self, pin, level, tick):
        """
        Callback that is called when a GPIO is toggled. For documentation on the
        usage of this callback by pigpio, refer to:
        http://abyz.me.uk/rpi/pigpio/python.html#callback
        
        Args:
            pin: the GPIO pin number (BCM)
            level: number that is either:
                        - 0 (falling edge),
                        - 1 (rising edge), or
                        - 2 (no change in level)
            tick: number of microseconds since boot
        """
        # NOTE: Consider using "tick" parameter for timing?
        t = time.time()

        if level == 1: # if this is a rising edge
            if pin == p.enc_l_a:
                t1 = self.most_recent_rising[p.enc_l_b] - self.most_recent_rising[p.enc_l_a]
                t2 = t - self.most_recent_rising[p.enc_l_b]
                self.left_direction_count += (t2 > t1) * 2 - 1
            elif pin == p.enc_l_b:
                t1 = self.most_recent_rising[p.enc_l_a] - self.most_recent_rising[p.enc_l_b]
                t2 = t - self.most_recent_rising[p.enc_l_a]
                self.left_direction_count += (t2 < t1) * 2 - 1
            elif pin == p.enc_r_a:
                t1 = self.most_recent_rising[p.enc_r_b] - self.most_recent_rising[p.enc_r_a]
                t2 = t - self.most_recent_rising[p.enc_r_b]
                self.right_direction_count += (t2 < t1) * 2 - 1
            elif pin == p.enc_r_b:
                t1 = self.most_recent_rising[p.enc_r_a] - self.most_recent_rising[p.enc_r_b]
                t2 = t - self.most_recent_rising[p.enc_r_a]
                self.right_direction_count += (t2 > t1) * 2 - 1

            self.most_recent_rising[pin] = t

        else:
            if pin == p.enc_l_a:
                t1 = self.most_recent_falling[p.enc_l_b] - self.most_recent_falling[p.enc_l_a]
                t2 = t - self.most_recent_falling[p.enc_l_b]
                self.left_direction_count += (t2 > t1) * 2 - 1
            elif pin == p.enc_l_b:
                t1 = self.most_recent_falling[p.enc_l_a] - self.most_recent_falling[p.enc_l_b]
                t2 = t - self.most_recent_falling[p.enc_l_a]
                self.left_direction_count += (t2 < t1) * 2 - 1
            elif pin == p.enc_r_a:
                t1 = self.most_recent_falling[p.enc_r_b] - self.most_recent_falling[p.enc_r_a]
                t2 = t - self.most_recent_falling[p.enc_r_b]
                self.right_direction_count += (t2 < t1) * 2 - 1
            elif pin == p.enc_r_b:
                t1 = self.most_recent_falling[p.enc_r_a] - self.most_recent_falling[p.enc_r_b]
                t2 = t - self.most_recent_falling[p.enc_r_a]
                self.right_direction_count += (t2 > t1) * 2 - 1
                
            self.most_recent_falling[pin] = t


        self.counts[pin] += 1

    def spin(self):
        r = rospy.Rate(p.encoder_freq)

        left_count = 0
        right_count = 0
        encoder_msg = Vector3()
        while not rospy.is_shutdown():
            left_count_new = (self.counts[p.enc_l_a] + self.counts[p.enc_l_b]) / 6. * np.sign(self.left_direction_count) * p.encoder_freq / p.gear_ratio
            right_count_new = (self.counts[p.enc_r_a] + self.counts[p.enc_r_b]) / 6. * np.sign(self.right_direction_count) * p.encoder_freq / p.gear_ratio

            left_count = self.alpha * left_count_new + (1 - self.alpha) * left_count
            right_count = self.alpha * right_count_new + (1 - self.alpha) * right_count

            encoder_msg.x = left_count
            encoder_msg.y = right_count

            self.publisher.publish(encoder_msg)

            for pin in p.encoder_pins:
                self.counts[pin] = 0

            self.left_direction_count = 0
            self.right_direction_count = 0

            r.sleep()




if __name__ == '__main__':
    e = Encoders()



