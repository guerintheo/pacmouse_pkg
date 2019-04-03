import RPi.GPIO as GPIO
import time
import pacmouse_pkg.src.params as p
import numpy as np
from multiprocessing import Process, Array
from collections import deque

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

class Encoders:
    def __init__(self):
        self.prevs = dict()
        self.most_recent_rising = dict()
        self.left_forward = True
        self.right_forward = True
        
        for i, pin in enumerate(p.encoder_pins):
            GPIO.setup(pin, GPIO.IN)
            GPIO.add_event_detect(pin, GPIO.BOTH, lambda p: self._callback(p))
            self.prevs[pin] = []
            self.most_recent_rising[pin] = False

    def _callback(self, pin):
        self.most_recent_rising[pin] = GPIO.input(pin)
        t = time.time()
        self.prevs[pin].append(t)
        # if pin == p.enc_l_b and len(self.prevs[p.enc_l_a]) > 1:
        #     time_between_different_pins = t - self.prevs[p.enc_l_a][-1]
        #     time_between_same_pin = self.prevs[p.enc_l_a][-1] - self.prevs[p.enc_l_a][-2]
        #     self.left_forward = (time_between_different_pins > time_between_same_pin/2.)
        #     # print self.left_forward
        # elif pin == p.enc_r_b and len(self.prevs[p.enc_r_a]) > 1:
        #     time_between_different_pins = t - self.prevs[p.enc_r_a][-1]
        #     time_between_same_pin = self.prevs[p.enc_r_a][-1] - self.prevs[p.enc_r_a][-2]
        #     self.right_forward = (time_between_different_pins > time_between_same_pin/2.)
        #     # print self.right_forward


    def speed(self, history=0.1):
        # TODO(izzy): estimate the direction of rotation by looking at the time diffs
        # between the a and b pins for each wheel
        
        t = time.time()
        counts = dict()

        for k in self.prevs.keys():
            prev = np.array(self.prevs[k])
            prev = prev[prev > t-history]
            counts[k] = prev.size
            self.prevs[k] = list(prev)

        len_l_a = len(self.prevs[p.enc_l_a])
        len_l_b = len(self.prevs[p.enc_l_b])



        if len_l_a > 2 and  len_l_b > 2:

            l_a_start_index = -min(len_l_a,len_l_b)
            l_b_start_index = l_a_start_index
            l_a_end_index = -1
            l_b_end_index = -1

            if not self.most_recent_rising[p.enc_l_a] == self.most_recent_rising[p.enc_l_b]:
                if self.prevs[p.enc_l_a][-1] > self.prevs[p.enc_l_b][-1]:
                    l_a_end_index = -2
                    l_b_start_index += 1
                else:
                    l_b_end_index = -2
                    l_a_start_index += 1

            diffs = np.array(self.prevs[p.enc_l_a][l_a_start_index:l_a_end_index]) -\
                    np.array(self.prevs[p.enc_l_b][l_b_start_index:l_b_end_index])

            print np.mean(diffs) > 0

            # print np.max(time_between_same_pin)/np.min(time_between_same_pin) 



        # speeds = np.array(((counts[p.enc_l_a] + counts[p.enc_l_b])/(2.*history*p.gear_ratio),
        #         (counts[p.enc_r_a] + counts[p.enc_r_b])/(2.*history*p.gear_ratio)))

        # directions =  (np.array([self.left_forward, self.right_forward], dtype=int) * 2 - 1)
        # velocities = directions * speeds
        # return velocities

    def stop(self):
        for pin in p.encoder_pins:
            GPIO.cleanup(pin)


if __name__ == '__main__':

    e = Encoders()

    while True:
        e.speed()
        time.sleep(0.1)

    e.stop()


