#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node communicate with the imu and publish its datas.
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
    rospy.init_node("quadruped_imu_node")

    pub = rospy.Publisher("/imu", Imu, queue_size=1, latch=True)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        msg = Imu()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"

        # orientation (quaternion)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # vitesse angulaire (rad/s)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        # accélération linéaire (m/s²)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        pub.publish(msg)

        rate.sleep()




# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    main()
