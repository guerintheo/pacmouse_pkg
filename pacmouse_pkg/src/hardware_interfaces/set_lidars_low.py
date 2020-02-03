import pigpio
import pacmouse_pkg.src.params as p

pi = pigpio.pi()  # handler for the Pi's GPIO pins

for pin in p.lidar_pins:
    pi.set_mode(pin,pigpio.OUTPUT)
    pi.write(pin, 0)  # 0 = low
    
pi.stop()
