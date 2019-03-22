# Raspberry-Pi Specific hardware setup

In order to prevent the motors from spinning at full chooch on boot, we need to set all GPIOs to output low on boot. 

Put the following line in `/boot/config.txt`. This will set GPIOs 0-27 to output driving low. This sets gpios as soon as the Pi boots, so the left motor will spin for 3 seconds before it stops. 
```
gpio=0-27=op,dl
```

**NOTE**: Due to a goof on the PCB, the GPIOs for Lidar 1 and 2 were swapped. The GPIOs for Lidar 3 and 4 were also swapped. We changed this in `paramys.py` but I'm just mentioning it here so that we remember. 
