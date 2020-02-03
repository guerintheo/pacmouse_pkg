from apa102_led import apa102
import time

NUM_LED = 7
MOSI = 8
SCLK = 7

strip = apa102.APA102(num_led=NUM_LED, mosi=MOSI, sclk=SCLK,order='rgb')

strip.clear_strip()

print("Brightness 0-100")
for b in range(0,5):
    strip.set_pixel_rgb(0,0xFF0000, b) #Red
    strip.set_pixel_rgb(1,0xFF7F00, b) #Orange
    strip.set_pixel_rgb(2,0xFFFF00, b) #Yellow
    strip.set_pixel_rgb(3,0x00FF00, b) #Green
    strip.set_pixel_rgb(4,0x0000FF, b) #Blue
    strip.set_pixel_rgb(5,0x4B0082, b) #Indigo
    strip.set_pixel_rgb(6,0x9400D3, b) #Violet
    strip.show()
    time.sleep(1)

strip.clear_strip()
strip.cleanup()

