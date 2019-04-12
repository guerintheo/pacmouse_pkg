#!/usr/bin/python
import rospy
import numpy as np
from pacmouse_pkg.src.utils.math_utils import wrap

from Adafruit_BNO055 import BNO055
from std_msgs.msg import Float64, Empty  # for heading value


class IMUNode(object):
    """
    Class that interfaces with the Adafruit BNO055 IMU.
    (https://www.adafruit.com/product/2472)
    """

    def __init__(self):
        rospy.init_node('imu_node')
        self.imu_pub = rospy.Publisher('/pacmouse/imu', Float64, queue_size=1)
        self.imu_am_upside_down = rospy.Publisher('/pacmouse/imu/am_upside_down', Empty, queue_size=1)

        rospy.Subscriber('/pacmouse/mode/zero_heading', Empty, self.reset_heading)

        self.bno = BNO055.BNO055(serial_port=rospy.get_param("/pacmouse/params/imu_serial_port"))
        # Try to initialize the IMU
        if not self.bno.begin():
            raise RuntimeError('The BNO055 failed to initialize. Check if the sensor is connected.')

        # Check the system status and test result
        status, self_test, error = self.bno.get_system_status()
        print('System status: {0}'.format(status))
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        if status == 0x01:
            # System status is in error mode
            print('System error: {0}'.format(error))
            print('See datasheet section 4.3.59 for the meaning.') 

    def spin(self):
        rate_hz = rospy.get_param("/pacmouse/params/imu_pub_rate")
        rate_ros = rospy.Rate(rate_hz)
        print('Reading from the IMU and publishing heading at rate of {} Hz...'.format(rate_hz))
        self.offset = np.array([0,0,0])
        while not rospy.is_shutdown():
            # Read the Euler angles for heading, roll, and pitch, all given in
            # degrees
            heading, roll, pitch = np.array(self.bno.read_euler()) - self.offset

            # TODO: Do something with roll or pitch data as a means of user input
            self.am_upside_down(roll, pitch)
            # Convert to radians
            heading = wrap(np.radians(heading))
            print('Heading: {} radians'.format(heading))
            heading_msg = Float64()
            heading_msg.data = heading
            self.imu_pub.publish(heading_msg)
            rate_ros.sleep()

    def am_upside_down(self, roll, pitch):
        # Needs to be both so you are fully upside down 
        am_upside_down = (-30 <= roll <= 30 and -120 <= pitch <= -60)
        if am_upside_down:
            # print('am_upside_down')
            self.imu_am_upside_down.publish(Empty())

    def reset_heading(self):
        print('offset heard')
        self.offset = np.array(self.bno.read_euler())

if __name__ == '__main__':
    imu = IMUNode()
    imu.spin()
