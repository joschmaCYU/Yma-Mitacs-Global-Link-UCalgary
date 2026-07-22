#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node controls the synchronisation betweeen all the nodes by publishing a tick signal on the node /control_tick.
Author : Florent Pralong
Date of creation : 10/06/2026
Version : 1.0
"""

# ---------------------------
# IMPORTS
# ---------------------------

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# Send the tick control signal at a specif rate and publish on /control_tick.
def main():
    rospy.init_node("quadruped_sync_clock_node")

    rate_hz = rospy.get_param("~rate", 5.0)
    pub = rospy.Publisher("/control_tick", Header, queue_size=1)

    rospy.loginfo("Waiting for first /joint_targets_rl message...")
    first_target = rospy.wait_for_message("/joint_targets_rl", JointState)
    rospy.loginfo("First /joint_targets_rl received. Starting control tick.")

    rate = rospy.Rate(rate_hz)
    seq = 0

    while not rospy.is_shutdown():
        msg = Header()
        msg.seq = seq
        msg.stamp = rospy.Time.now()
        msg.frame_id = "control_tick"

        pub.publish(msg)

        seq += 1
        rate.sleep()

# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    main()
