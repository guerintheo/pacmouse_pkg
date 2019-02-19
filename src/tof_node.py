#!/usr/bin/env python

import rospy
from pacmouse_pkg.msg import Lidars
from subprocess import Popen, PIPE
import numpy as np

if __name__ == '__main__':
    pub = rospy.Publisher('/tof', Lidars, queue_size=1)
    rospy.init_node('tof_node')
    rate = rospy.Rate(100) # 100hz
    print "Opening ./tof_reader"
    tof_process = Popen(['./tof_reader'], stdout=PIPE, stderr=PIPE) # start the tof sensors 
    
    while not rospy.is_shutdown():
        data = Lidars()
        data.header.stamp = rospy.Time.now()
        try:
            raw_dists = tof_process.stdout.readline().strip()
            data.ranges = np.array([int(i) for i in raw_dists.split('\t')])/255.
            print data.ranges
            pub.publish(data)
        except:
            print raw_dists
