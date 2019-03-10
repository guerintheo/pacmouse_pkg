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
        
        for i, pin in enumerate(p.encoder_pins):
            GPIO.setup(pin, GPIO.IN)
            GPIO.add_event_detect(pin, GPIO.BOTH, lambda p: self._callback(p))
            self.prevs[pin] = []

    def _callback(self, pin):
        # rising = GPIO.input(pin)
        self.prevs[pin].append(time.time())


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

        return ((counts[p.enc_l_a] + counts[p.enc_l_b])/(2.*history*p.gear_ratio),
                (counts[p.enc_r_a] + counts[p.enc_r_b])/(2.*history*p.gear_ratio))

    def stop(self):
        for pin in p.encoder_pins:
            GPIO.cleanup(pin)


if __name__ == '__main__':

    e = Encoders()

    while True:
        print e.speed()
        time.sleep(0.1)

    e.stop()


