import RPi.GPIO as GPIO
import time
import pacmouse_pkg.src.params as p
import numpy as np
from multiprocessing import Process, Array

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

class Encoders:
    def __init__(self):
        for pin in p.encoder_pins:
            GPIO.setup(pin, GPIO.IN)
        self.sample_duration = 0.1


    def get_speed(self):
        start = time.time()
        counts = np.zeros(4)
        times = np.zeros(4)
        corresponding = [1, 0, 3, 2]
        prev_times = np.ones(4) * time.time()
        prevs = np.array([GPIO.input(pin) for pin in p.encoder_pins])

        while time.time() - start < self.sample_duration:
            t = time.time()
            # get the newest encoder value
            vals = np.array([GPIO.input(pin) for pin in p.encoder_pins])

            # see which pins have changed state
            changed = (vals != prevs)

            # # update the most recent flip times for the encoders that have chagned
            # prev_times[changed] = t

            # times[corresponding[changed]] += t - prevtimes[changed]

            # # increment the counts for the encoders that have changed
            counts += changed



            # set the previous encoder value
            prevs = vals


        return (counts[0] + counts[1])/2./self.sample_duration, \
               (counts[2] + counts[3])/2./self.sample_duration


class AsyncEncoders:
    def __init__(self):
        self.sample_duration = 0.1
        self.data = Array('i', np.zeros(SAMPLE_ARRAY_SIZE + 1, dtype=int))
        self.sampler_process = Process(target=self.run, name='encoders', args=(data,))
        self.sampler_process.start()
        self.sampler_process.join()


    def run(self, data):
        for pin in p.encoder_pins:
            GPIO.setup(pin, GPIO.IN)


        for pin in p.encoder_pins:
            GPIO.cleanup(GPIO.IN)

    def get_speed(self):
        start = time.time()
        counts = np.zeros(4)
        times = np.zeros(4)
        corresponding = [1, 0, 3, 2]
        prev_times = np.ones(4) * time.time()
        prevs = np.array([GPIO.input(pin) for pin in p.encoder_pins])

        while time.time() - start < self.sample_duration:
            t = time.time()
            # get the newest encoder value
            vals = np.array([GPIO.input(pin) for pin in p.encoder_pins])

            # see which pins have changed state
            changed = (vals != prevs)

            # # update the most recent flip times for the encoders that have chagned
            # prev_times[changed] = t

            # times[corresponding[changed]] += t - prevtimes[changed]

            # # increment the counts for the encoders that have changed
            counts += changed



            # set the previous encoder value
            prevs = vals


        return (counts[0] + counts[1])/2./self.sample_duration, \
               (counts[2] + counts[3])/2./self.sample_duration



if __name__ == '__main__':

    e = Encoders()
    while True:
        print e.get_speed()



