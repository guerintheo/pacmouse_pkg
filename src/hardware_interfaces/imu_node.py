#!/usr/bin/python
import rospy
import numpy as np
from Adafruit_BNO055 import BNO055
from std_msgs.msg import Float64  # for heading value


class IMUNode(object):
    """
    Class that interfaces with the Adafruit BNO055 IMU.
    (https://www.adafruit.com/product/2472)
    """
    
    def __init__(self):
        rospy.init_node('imu_node')
        self.imu_pub = rospy.Publisher('/pacmouse/imu', Float64, queue_size=1)
        self.bno = BNO055.BNO055(serial_port=rospy.get_param("/pacmouse_pkg/params/imu_serial_port"))
        
        # Try to initialize the IMU
        if not bno.begin():
            raise RuntimeError('The BNO055 failed to initialize. Check if the sensor is connected.')
        
        # Check the system status and test result
        status, self_test, error = bno.get_system_status()
        print('System status: {0}'.format(status))
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        if status == 0x01:
            # System status is in error mode
            print('System error: {0}'.format(error))
            print('See datasheet section 4.3.59 for the meaning.')
            
    def spin(self):
        rate_hz = rospy.get_param("/pacmouse_pkg/params/imu_pub_rate")
        rate_ros = rospy.Rate(rate_hz)
        print('Reading from the IMU and publishing heading at rate of {} Hz...'.format(rate_hz))
        while not rospy.is_shutdown():
            # Read the Euler angles for heading, roll, and pitch, all given in
            # degrees
            heading, roll, pitch = bno.read_euler()
            # TODO: Do something with roll or pitch data as a means of user input
            # Convert to radians
            heading = np.radians(heading)
            print('Heading: {} radians'.format(heading))
            heading_msg = Float64()
            heading_msg.data = heading
            self.imu_pub.publish(heading_msg)
            rate_ros.sleep()
            

if __name__ == '__main__':
    imu = IMUNode()
    imu.spin()