#!/usr/bin/env python

# test-APA102.py
# 2017-03-28
# Public Domain

import time
import sys
import random

LEDS=30

WAVES=0
PIGPIO=1
SPIDEV=2

DAT=26
CLK=8

DATB=(1<<DAT)
CLKB=(1<<CLK)

"""
Start Frame  0000 0000 | 0000 0000 | 0000 0000 | 0000 0000

LED Frame 1  111b bbbb | BBBB BBBB | GGGG GGGG | RRRR RRRR
...
LED Frame N  111b bbbb | BBBB BBBB | GGGG GGGG | RRRR RRRR

End Frame    1111 1111 | 1111 1111 | 1111 1111 | 1111 1111

b bbbb     Brightness 0-31
BBBB BBBB  Blue 0-255
GGGG GGGG  Green 0-255
RRRR RRRR  Red 0-255
"""

apa102_cmd=[0]*4 + [0xe1,0, 0, 0]*LEDS + [255]*4
chain=[None]*(2*len(apa102_cmd))
gwid=[None]*16

def create_byte_waves():
   for i in range(16):
      pulse=[]
      for bit in range(4):
         if (1<<(3-bit)) & i: # 1 bit
            pulse.append(pigpio.pulse(DATB, CLKB, 1))
         else: # 0 bit
            pulse.append(pigpio.pulse(0, DATB|CLKB, 1))
         pulse.append(pigpio.pulse(CLKB, 0, 1))
      pi.wave_add_generic(pulse)
      gwid[i] = pi.wave_create()

def tx_bytes(bytes):
   global chain
   while pi.wave_tx_busy():
      pass
   j = 0
   for i in range(len(bytes)):
      chain[j] = gwid[(bytes[i]>>4)&15]
      j += 1
      chain[j] = gwid[bytes[i]&15]
      j += 1
   pi.wave_chain(chain)

def update():
   if method == SPIDEV:
      spi.xfer(apa102_cmd)
   elif method == PIGPIO:
      pi.spi_xfer(h, apa102_cmd)
   else:
      tx_bytes(apa102_cmd)

def set_LED_RGB(led, r, g, b):
   offset = (led*4) +4
   apa102_cmd[offset+1] = b
   apa102_cmd[offset+2] = g
   apa102_cmd[offset+3] = r

def set_LED_PRGB(led, p, r, g, b):
   offset = (led*4) +4
   apa102_cmd[offset  ] = 0xE0 + p
   apa102_cmd[offset+1] = b
   apa102_cmd[offset+2] = g
   apa102_cmd[offset+3] = r

def blobs():

   fpos=[0.0, LEDS/2.0, LEDS-1.0, 0.0, LEDS/2.0, LEDS-1.0]

   nblobs = len(fpos)

   ipos=[0.0]*nblobs

   inc=[0.1, 0.11, -0.12, 0.13, 0.14, -0.15]

   colour=[
      [200,  0,   0],
      [0,  200,   0],
      [0,   0,  200],
      [200, 200,  0],
      [200,  0, 200],
      [0,  200, 200]
   ]

   for i in range(nblobs):
      ipos[i] = int(fpos[i])

   while time.time() < finish:

      # Clear old slots.
      for i in range(nblobs):
         set_LED_RGB(ipos[i], 0, 0, 0)

      # Calculate new positions.
      for i in range(nblobs):

         fpos[i] += inc[i]
         ipos[i] = int(fpos[i])

         if ipos[i] < 0:
            ipos[i] = 0
            inc[i] = -inc[i]
         elif ipos[i] >= LEDS:
            ipos[i] = LEDS - 1
            inc[i] = -inc[i]

      # Set new slots.
      for i in range(nblobs):
         set_LED_RGB(ipos[i], colour[i][0], colour[i][1], colour[i][2])

      # Refresh LEDs
      update()

      time.sleep(0.02)

def bounce():

   fpos=[0.0, LEDS/2.0, LEDS-1.0]

   nblobs = len(fpos)

   inc=[0.1, 0.15, -0.2]

   colour=[
      [200,   0,   0],
      [  0, 200,   0],
      [  0,   0, 200],
   ]

   ipos=[0.0]*nblobs
   rev=[False]*nblobs

   for i in range(nblobs):
      ipos[i] = int(fpos[i])

   while time.time() < finish:

      # Clear old slots.
      for i in range(nblobs):
         set_LED_RGB(ipos[i], 0, 0, 0)

      # Calculate new positions.
      for i in range(nblobs):

         fpos[i] += inc[i]
         ipos[i] = int(fpos[i])

         if ipos[i] < 0:
            ipos[i] = 0
            inc[i] = -inc[i]
         elif ipos[i] >= LEDS:
            ipos[i] = LEDS - 1
            inc[i] = -inc[i]

      # Check for collisions.
      for i in range(nblobs):
         rev[i] = False

      for i in range(nblobs):
         for j in range(i+1,nblobs):
            if (ipos[i] > 0) and (ipos[i] != (LEDS-1)):
               if ipos[i] == ipos[j]:
                  rev[i] = True
                  rev[j] = True

      for i in range(nblobs):
         if rev[i]:
            inc[i] = -inc[i]

      # Set new slots.
      for i in range(nblobs):
         set_LED_RGB(ipos[i], colour[i][0], colour[i][1], colour[i][2])

      # Refresh LEDs
      update()

      time.sleep(0.02)

def clock():

   R = 128
   G = 0
   B = 0

   s = -1

   while time.time() < finish:

      now = time.localtime()

      s_now = now[5] % 2

      if s_now != s:

         s = s_now

         H = now[3] / 10
         h = now[3] % 10

         M = now[4] / 10
         m = now[4] % 10

         # Clear LEDs.
         for i in range(LEDS):
            set_LED_RGB(i, 0, 0, 0)

         for i in range(H):
            set_LED_RGB(i+1, R, G, B)

         for i in range(h):
            set_LED_RGB(i+4, R, G, B)

         for i in range(M):
            set_LED_RGB(i+14, R, G, B)

         for i in range(m):
            set_LED_RGB(i+20, R, G, B)

         set_LED_RGB( 0, 0, 8, 0)
         set_LED_RGB( 3, 0, 8, 0)
         set_LED_RGB(13, 0, 8, 0)
         set_LED_RGB(19, 0, 8, 0)

         if s:
            set_LED_PRGB(29, 2, 0, 8, 0)

         # Refresh LEDs
         update()

      time.sleep(0.1)

def xylon():

   slots = 8
   fpos = 0
   ipos = 0
   inc = 0.3

   change_time = time.time() + 10.0

   R = 128
   G = 0
   B = 0

   while time.time() < finish:

      # Calculate new position.
      fpos += inc
      if ipos != int(fpos):

         # Clear old slots.
         for i in range(slots):
            if 0 <= (ipos + i) < LEDS:
               set_LED_RGB(ipos+i, 0, 0, 0)

         ipos = int(fpos)

         if ipos < -slots:
            ipos = -slots
            inc = -inc
         elif ipos >= LEDS:
            ipos = LEDS - 1
            inc = -inc

         # Set new slots.
         if inc > 0:
            for i in range(slots):
               if 0 <= (ipos + i) < LEDS:
                  #set_LED_RGB(ipos+i, (1<<i)+1, 0, 0)
                  set_LED_PRGB(ipos+i, i+1, R, G, B)
         else:
            for i in range(slots):
               if 0 <= (ipos + i) < LEDS:
                  #set_LED_RGB(ipos+i, (1<<(slots-1-i))+1, 0, 0)
                  set_LED_PRGB(ipos+i, slots-i, R, G, B)

         # Refresh LEDs
         update()

      if time.time() > change_time:
         change_time += 10
         R = random.randint(0, 255)
         G = random.randint(0, 255)
         B = random.randint(0, 255)

      time.sleep(0.02)

# -------------------------------------------------------------------

runtime=1e6

if len(sys.argv) > 1:
   cmd = int(sys.argv[1])
else:
   cmd = 0

if len(sys.argv) > 2:
   method = int(sys.argv[2])
   if method < WAVES or method > SPIDEV:
      method = WAVES
else:
   method = WAVES

if method == PIGPIO or method == WAVES:
   import pigpio
   pi = pigpio.pi()

   if not pi.connected:
      exit()

   if method == PIGPIO:
      h = pi.spi_open(0, 2e6, 0xE0) # 0xE0 says not to set chip enables
   else:
      oldDATmode = pi.get_mode(DAT)
      oldCLKmode = pi.get_mode(CLK)
      pi.set_mode(DAT, pigpio.OUTPUT)
      pi.set_mode(CLK, pigpio.OUTPUT)
      create_byte_waves()
else:
   import spidev
   spi = spidev.SpiDev()
   spi.open(0, 0)
   spi.max_speed_hz = int(2e6)

finish = time.time() + runtime

if 1 <= cmd <= 4:

   if cmd == 1:
      s = "blobs"
   elif cmd == 2:
      s = "bounce"
   elif cmd == 3:
      s = "clock"
   else: # cmd == 4
      s = "xylon"

   s += " using "

   if method == WAVES:
      s += "pigpio waves"
   elif method == PIGPIO:
      s += "pigpio SPI"
   else:
      s += "spidev SPI"

   print(s)

   try:
      if cmd == 1:
         blobs()
      elif cmd == 2:
         bounce()
      elif cmd == 3:
         clock()
      else: # cmd == 4
         xylon()

   except KeyboardInterrupt:
      print("")
      for i in range(LEDS):
         set_LED_RGB(i, 0, 0, 0)
      update()
else:

   print("Usage: ./test-APA102.py n [m]\nwhere n is one of:")
   print("   1=blobs, 2=bounce, 3=clock, 4=xylon")
   print("\nand the optional method m is one of:")
   print("   0=pigpio waves(default), 1=pigpio SPI, 2=spidev SPI")


if method == PIGPIO or method == WAVES:
   if method == PIGPIO:
      pi.spi_close(h)
   else:
      while pi.wave_tx_busy():
         pass
      for w in gwid:
         pi.wave_delete(w)
      pi.set_mode(DAT, oldDATmode)
      pi.set_mode(CLK, oldCLKmode)
   pi.stop()
else:
   spi.close()

