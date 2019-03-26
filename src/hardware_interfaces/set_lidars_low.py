import RPi.GPIO as GPIO
import pacmouse_pkg.src.params as p

GPIO.setmode(GPIO.BOARD)

for pin in p.lidar_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
    # GPIO.cleanup(PIN)
