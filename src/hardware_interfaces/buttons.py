import RPi.GPIO as GPIO
import time
import pacmouse_pkg.src.params as p

GPIO.setmode(GPIO.BOARD)

for pin in p.button_pins:
	GPIO.setup(pin, GPIO.IN, GPIO.PUD_UP)

while True:
	vals = [GPIO.input(pin) for pin in p.button_pins]
	print vals
	time.sleep(0.1)
