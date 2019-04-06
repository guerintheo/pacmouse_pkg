import pigpio
import pacmouse_pkg.src.params as p
import time
import numpy as np
# from pacmouse_pkg.src.hardware_interfaces.encoders import Encoders
from pacmouse_pkg.src.estimation_control.control import PID
from multiprocess import Process, Array

class Motors:
    def __init__(self):
        self.pi = pigpio.pi()  # handler for Pi's GPIO pins
        
        for pin in p.motor_pins:
            self.pi.set_mode(pin, pigpio.OUTPUT)

        self.pi.write(p.motor_mode_pin, 1)  # 1=high

        # intialize the PWM for the motors
        self.pi.set_PWM_frequency(p.ml_pwm, p.motor_pwm_freq)
        self.pi.set_PWM_frequency(p.mr_pwm, p.motor_pwm_freq)

    def set(self, l, r):
        assert -1 <= l <= 1
        assert -1 <= r <= 1

        self.l = l
        self.r = r

        # set the directions of the motors. 0 is forward, 1 is backward
        self.pi.write(p.ml_dir, int(self.l < 0))
        self.pi.write(p.mr_dir, int(self.r < 0))

        # set the speeds of the motors. 255 dutycycle is highest, 0 is PWM off
        self.pi.set_PWM_dutycycle(p.ml_pwm, abs(self.l)*255.)
        self.pi.set_PWM_dutycycle(p.mr_pwm, abs(self.r)*255.)

    def stop(self):
        self.set(0,0)
        # To "release pigpio resources" (http://abyz.me.uk/rpi/pigpio/python.html#stop):
        self.pi.stop()


if __name__ == '__main__':
    m = Motors()
    print 'ramp speed'

    for i in np.linspace(-1, 1, 200):
        m.set(i, i)
        time.sleep(0.05)
    m.stop()


