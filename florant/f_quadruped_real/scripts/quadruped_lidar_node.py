#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node communicate with the liddar and publish its datas. 
Author : 
Date of creation : 29/06/2026
Version : 1.0
"""

# ---------------------------
# IMPORTS
# ---------------------------

import sys
import termios
import tty
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32MultiArray


def main():
    rospy.init_node("quadruped_lidar_node")

    pub = rospy.Publisher("/lidar", LaserScan, queue_size=1, latch=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        msg = LaserScan()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "laser"

        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 0.01745  # 1 deg

        msg.time_increment = 0.0
        msg.scan_time = 0.1

        msg.range_min = 0.1
        msg.range_max = 30.0

        # 180 mesures à 2 m
        msg.ranges = [2.0] * 180

        pub.publish(msg)

        rate.sleep()


# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    main()
