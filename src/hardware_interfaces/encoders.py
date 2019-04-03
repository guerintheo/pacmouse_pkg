import RPi.GPIO as GPIO
import time
import rospy
from geometry_msgs.msg import Vector3  # TODO: Use a better message type
import pacmouse_pkg.src.params as p
import numpy as np
from multiprocessing import Process, Array
from collections import deque

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

class Encoders:
    def __init__(self):
        rospy.init_node('encoder_node')
        self.publisher = rospy.Publisher('/pacmouse/encoders', Vector3, queue_size=1)

        self.counts = dict()
        self.most_recent_rising = dict()
        self.most_recent_falling = dict()
        self.left_direction_count = 0
        self.right_direction_count = 0
        self.alpha = 0.5
        
        for i, pin in enumerate(p.encoder_pins):
            GPIO.setup(pin, GPIO.IN)
            GPIO.add_event_detect(pin, GPIO.BOTH, lambda p: self._callback(p))
            self.counts[pin] = 0
            self.most_recent_rising[pin] = time.time()
            self.most_recent_falling[pin] = time.time()

        self.spin()

    def _callback(self, pin):
        t = time.time()

        if GPIO.input(pin): # if this is a rising edge
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
            left_count_new = (self.counts[p.enc_l_a] + self.counts[p.enc_l_b]) / 2. * np.sign(self.left_direction_count)
            right_count_new = (self.counts[p.enc_r_a] + self.counts[p.enc_r_b]) / 2. * np.sign(self.right_direction_count)

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



