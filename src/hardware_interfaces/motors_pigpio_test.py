#!/usr/bin/python

# Before running this script, make sure you have run `sudo pigpiod` to launch
# the pigpio daemon

import pigpio  # to control the motors with hardware-timed PWM
import RPi.GPIO as GPIO  # use for the non-PWM GPIO tasks.  # TODO: Can probably just use pigpio to do this
import pacmouse_pkg.src.params as p
import time

####### SETUP

pi = pigpio.pi()
bcm_ml_pwm = 12  # corresponds to pin 32 on Pi, motor left PWM  # TODO: Put in params.py if this works
bcm_mr_pwm = 13  # corresponds to pin 33 on Pi, motor right PWM  # TODO: Put in params.py if this works

GPIO.setmode(GPIO.BOARD)
GPIO.setup(p.ml_dir, GPIO.OUT)
GPIO.setup(p.mr_dir, GPIO.OUT)
GPIO.setup(p.motor_mode_pin, GPIO.OUT)

GPIO.output(p.motor_mode_pin, GPIO.HIGH)

pi.set_PWM_frequency(bcm_ml_pwm, p.motor_pwm_freq)
pi.set_PWM_frequency(bcm_mr_pwm, p.motor_pwm_freq)


######## PWM CONTROL OF MOTORS

# pi.set_servo_pulsewidth(bcm_ml_pwm, 2500)  # not the function we want
# pi.set_servo_pulsewidth(bcm_mr_pwm, 1700)

pi.set_PWM_dutycycle(bcm_ml_pwm, 255)  # the function we do want

time.sleep(3)

pi.set_PWM_dutycycle(bcm_ml_pwm, 0)
