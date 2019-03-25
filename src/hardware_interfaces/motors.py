import RPi.GPIO as GPIO
import pacmouse_pkg.src.params as p
import time
import numpy as np
from pacmouse_pkg.src.hardware_interfaces.encoders import Encoders
from pacmouse_pkg.src.estimation_control.control import PID
from multiprocess import Process, Array

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
		self.set(0,0)
		for pin in p.motor_pins:
			GPIO.cleanup(pin)



class MotorController:
	def __init__(self, freq=p.motor_controller_freq):
		self.delay = 1./freq
		self._data = Array('f', np.zeros(5, dtype=float))
		self._process = Process(target=self._run, name='Motor Controller', args=(self._data,))
		self._process.start()

	def set(self, rpm_L, rpm_R):
		if self._data.acquire():
			self._data[0] = rpm_L
			self._data[1] = rpm_R
			self._data.release()

	def get(self):
		if self._data.acquire():
			rpm = self._data[2:4]
			self._data.release()
		return np.array(rpm)

	def stop(self):
		if self._data.acquire():
			self._data[4] = 1
			self._data.release()
		time.sleep(self.delay*3)
		self._process.terminate()

	def _run(self, data):
		motors = Motors()
		encoders = Encoders()
		pid_L = PID(*p.motor_controller_pid, control_range=[-1,1])
		pid_R = PID(*p.motor_controller_pid, control_range=[-1,1])
		sp = data[:2]
		current = data[2:4]
		stop = data[4]
		while stop == 0:
			current = encoders.speed(history=0.5) # read the current speed of the motors

			# non-blocking interaction with the shared array
			if data.acquire(False):
				sp = data[:2]
				data[2:4] = current
				stop = data[4]
				data.release()

			# step the PIDs and send the command to the motors
			err_L, err_R = sp - np.sign(sp)*current
			cmd_L = pid_L.step(err_L, self.delay)
			cmd_R = pid_R.step(err_R, self.delay)
			# print err_L, err_R, cmd_L, cmd_R
			motors.set(cmd_L, cmd_R)

			time.sleep(self.delay)


		motors.stop()
		encoders.stop()
		print('Motor controller subprocess terminated')




if __name__ == '__main__':
	m = Motors()

	print 'ramp speed'

	for i in np.linspace(-1, 1., 100):
		# print i, -i
		m.set(i, i)
		# print m.get()
		time.sleep(0.1)
	m.stop()

	# m.set(1,1)
	# print 'fixed speed'

	# for i in range(100):
	# 	print m.get()
	# 	time.sleep(0.1)
	# m.stop()
	# m = MotorController()
	# time.sleep(1)
	# # m.set([-1,-1])
	# # time.sleep(1)

	# m.set([.3,.3])
	# time.sleep(5)
	# # m.set([5,0])
	# # time.sleep(1)
	# m.set([0,0])
	# time.sleep(1)
	# m.stop()
