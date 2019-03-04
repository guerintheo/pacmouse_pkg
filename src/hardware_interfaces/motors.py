import RPi.GPIO as GPIO
import pacmouse_pkg.src.params as p
import time
import numpy as np

GPIO.setmode(GPIO.BOARD)

class Motors:
	def __init__(self):
		for pin in p.motor_pins:
			GPIO.setup(pin, GPIO.OUT)

		GPIO.output(p.motor_mode_pin, GPIO.HIGH)

		# intialize the PWM for the motors
		self.ml = GPIO.PWM(p.ml_pwm, p.motor_pwm_freq)
		self.mr = GPIO.PWM(p.mr_pwm, p.motor_pwm_freq)
		self.ml.start(0)
		self.mr.start(0)

	def set(self, l, r):
		assert -1 <= l <= 1
		assert -1 <= r <= 1

		self.l = l
		self.r = r

		# set the directions of the motors
		GPIO.output(p.ml_dir, int(self.l >= 0))
		GPIO.output(p.mr_dir, int(self.r >= 0))

		# set the speeds of the motors
		self.ml.ChangeDutyCycle(abs(self.l)*100.)
		self.mr.ChangeDutyCycle(abs(self.r)*100.)

	def stop(self):
		for pin in p.motor_pins:
			GPIO.cleanup(pin)

if __name__ == '__main__':
	m = Motors()

	for i in np.linspace(-1., 1., 100):
		print i, -i
		m.set(i, -i)
		time.sleep(0.1)

	m.set(0,0)
	m.stop()
