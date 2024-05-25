#!/usr/bin/env python

import tsys01_driver as tsys
import rospy
import time
import numpy as np
from sensor_msgs.msg import Temperature

if __name__ == '__main__':
    try:
        # set up ros stuff
        rospy.init_node('tsys01_node')
        bus = rospy.get_param('~bus', '1')
	temp_variance = rospy.get_param('~temp_variance', 0.001)
        pubTemp = rospy.Publisher('temperature/water/accurate',Temperature, queue_size=1)
        rate = rospy.Rate(100)  # 100Hz data read
        sensor = tsys.TSYS01(int(bus))  # Default I2C bus is 1 (Raspberry Pi 3)

        # sensor.init must run immediately after installation of tsys01 object
        sensor.init()

        while not rospy.is_shutdown():
            sensor.read()
            tempC = sensor.temperature(tsys.UNITS_Centigrade)
            
            # update messages
            temp_msg = Temperature()
            temp_msg.header.stamp = rospy.Time.now()
            temp_msg.header.frame_id = 'temperature_data'
            temp_msg.temperature = float(tempC)
            temp_msg.variance = float(temp_variance)
            pubTemp.publish(temp_msg)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
