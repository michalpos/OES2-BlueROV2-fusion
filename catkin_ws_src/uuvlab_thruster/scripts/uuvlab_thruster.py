#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from thruster_manager import ThrusterManager
from geometry_msgs.msg import Wrench


def get_thruster_input(msg):
	thrust_force = np.array([msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z])
	thrust_pwm = TM.sendThrustInput(msg)


if __name__ == '__main__':
	rospy.init_node('thruster_manager',anonymous=True)
	TM = ThrusterManager()
	rospy.Subscriber("thruster_manager/input", Wrench, get_thruster_input)
	rospy.spin()
