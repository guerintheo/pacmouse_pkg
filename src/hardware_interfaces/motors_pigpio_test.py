#!/usr/bin/python

# Before running this script, make sure you have run `sudo pigpiod` to launch
# the pigpio daemon. Can run `sudo killall pigpiod` to kill the daemon.

import pigpio  # to control the motors with hardware-timed PWM
#import RPi.GPIO as GPIO  # use for the non-PWM GPIO tasks.  # TODO: Can probably just use pigpio to do this
import pacmouse_pkg.src.params as p
import time

####### SETUP

pi = pigpio.pi()

# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(p.ml_dir, GPIO.OUT)
# GPIO.setup(p.mr_dir, GPIO.OUT)
# GPIO.setup(p.motor_mode_pin, GPIO.OUT)
# 
# GPIO.output(p.motor_mode_pin, GPIO.HIGH)

for pin in p.motor_pins:
    pi.set_mode(pin, pigpio.OUTPUT)

pi.write(p.motor_mode_pin, 1)  # 1=high

pi.set_PWM_frequency(p.ml_pwm, p.motor_pwm_freq)
pi.set_PWM_frequency(p.mr_pwm, p.motor_pwm_freq)


######## PWM CONTROL OF MOTORS

# pi.set_servo_pulsewidth(bcm_ml_pwm, 2500)  # not the function we want
# pi.set_servo_pulsewidth(bcm_mr_pwm, 1700)

pi.write(p.ml_dir, 0)
pi.set_PWM_dutycycle(p.ml_pwm, 100)  # the function we do want. 255 dutycycle is highest
time.sleep(1)
pi.set_PWM_dutycycle(p.ml_pwm, 0)
time.sleep(1)

# Change motor direction
pi.write(p.ml_dir, 1)
pi.set_PWM_dutycycle(p.ml_pwm, 100)  # the function we do want
time.sleep(1)
pi.set_PWM_dutycycle(p.ml_pwm, 0)
