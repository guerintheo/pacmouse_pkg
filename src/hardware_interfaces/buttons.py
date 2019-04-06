import pigpio
import time
import pacmouse_pkg.src.params as p

pi = pigpio.pi()  # handler for Pi's GPIO pins

for pin in p.button_pins:
    pi.set_mode(pin, pigpio.INPUT)
    pi.set_pull_up_down(pin, pigpio.PUD_UP)

while True:
    vals = [pi.read(pin) for pin in p.button_pins]
	print vals
	time.sleep(0.1)
    
pi.stop()
