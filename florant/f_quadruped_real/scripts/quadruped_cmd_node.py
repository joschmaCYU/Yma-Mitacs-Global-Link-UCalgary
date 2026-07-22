#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node receive user commands and publish them.
Author : 
Date of creation : 29/06/2026
Version : 1.0
"""

# ---------------------------
# IMPORTS
# ---------------------------

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32MultiArray



def main():
    rospy.init_node("quadruped_cmd_node")

    pub = rospy.Publisher("/cmd", Float32MultiArray, queue_size=1, latch=True)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        msg = Float32MultiArray()
        msg.data = [1.0, 2.0, 3.0, 4.0]  # valeurs dummy

        pub.publish(msg)

        rate.sleep()


# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    main()
