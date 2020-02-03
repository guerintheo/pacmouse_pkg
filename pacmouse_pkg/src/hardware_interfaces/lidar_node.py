#!/usr/bin/python

import rospy
import pacmouse_pkg.src.params as p
from std_msgs.msg import Bool
from subprocess import Popen, PIPE
import numpy as np
from pacmouse_pkg.msg import Lidars # TODO: Is this the best message type?



class LIDARs(): 
    def __init__(self):
        rospy.init_node('lidar_node')
        self.publisher = rospy.Publisher('/pacmouse/lidars', Lidars, queue_size=1)

        self.tof_array = None 
        self.tof_process = Popen(['/home/pi/ros_catkin_ws/src/pacmouse_pkg/src/hardware_interfaces/tof_test'], stdout=PIPE, stderr=PIPE) # start the tof sensors
        print 'Launched tof_test at PID', self.tof_process.pid

        rospy.on_shutdown(self.close) 

        self.spin()

    def spin(self):
        r = rospy.Rate(20)
        msg = Lidars()
        while self.tof_process is not None:
            raw_dists = None
            try:
                raw_dists = self.tof_process.stdout.readline().strip()
                self.tof_array = np.array([int(i) for i in raw_dists.split('\t')])
                
                msg.dists[:p.num_lidars] = self.tof_array
                # print msg
                self.publisher.publish(msg)
            except:
                if raw_dists is not None: print raw_dists
            r.sleep()

    def close(self):
        self.tof_process.kill()
        self.tof_process = None


if __name__ == "__main__":
    l = LIDARs()