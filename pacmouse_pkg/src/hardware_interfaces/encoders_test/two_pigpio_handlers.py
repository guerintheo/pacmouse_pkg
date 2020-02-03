import pigpio
import time

import pacmouse_pkg.src.params as p




##### SETUP

pi1 = pigpio.pi()  # handler for Pi's GPIO pins
pi2 = pigpio.pi()

for pin in p.motor_pins:
    pi1.set_mode(pin, pigpio.OUTPUT)
    pi2.set_mode(pin, pigpio.OUTPUT)

pi1.write(p.motor_mode_pin, 1)  # 1=high
pi2.write(p.motor_mode_pin, 1)

# intialize the PWM for the motors
pi1.set_PWM_frequency(p.ml_pwm, p.motor_pwm_freq)
pi1.set_PWM_frequency(p.mr_pwm, p.motor_pwm_freq)
pi2.set_PWM_frequency(p.ml_pwm, p.motor_pwm_freq)
pi2.set_PWM_frequency(p.mr_pwm, p.motor_pwm_freq)


##### TESTING

time.sleep(.1)

pi1.write(p.mr_dir, 0)
pi2.write(p.mr_dir, 0)
pi1.set_PWM_dutycycle(p.mr_pwm, 200.)
pi2.set_PWM_dutycycle(p.mr_pwm, 200.)

time.sleep(2)

pi1.set_PWM_dutycycle(p.mr_pwm, 0.)
pi2.set_PWM_dutycycle(p.mr_pwm, 0.)
pi1.stop()
pi2.stop()