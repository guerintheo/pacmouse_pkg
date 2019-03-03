import RPi.GPIO as GPIO
import time

pins = [12,35,38,40]

GPIO.setmode(GPIO.BOARD)

for pin in pins:
	GPIO.setup(pin, GPIO.IN, GPIO.PUD_DOWN)

while True:
	vals = [GPIO.input(pin) for pin in pins]
	print vals
	time.sleep(0.1)
