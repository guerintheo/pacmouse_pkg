#!/usr/bin/python
import rospy
import numpy as np
import enums
from msg import AgentState, LightState


def main():
    # ROS Setup
    ###########
    rospy.init_node("light_state_publisher")
    # Publishers
    ############
    light_state_pub = rospy.Publisher('/pacmouse/light_state', LightState, queue_size=1)

    light_state = LightState()

    #TODO: choose publishing rate
    r = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():

        # TODO: update light_state parameters

        light_state_pub.publish(light_state)

        r.sleep()




if __name__ == '__main__':
    main()
