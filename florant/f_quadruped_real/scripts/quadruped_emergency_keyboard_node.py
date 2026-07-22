#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node contain the emergency stop triggered by a keyboard shortcut ( ctrl + alt + z) or pressing space key when you are in the terminal windows of this script. 
Author : Florent Pralong
Date of creation : 15/06/2026
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

# Publish emergency stop on the /emergency_stop topic 
def main():
    rospy.init_node("quadruped_emergency_keyboard_node")

    pub = rospy.Publisher("/emergency_stop", Bool, queue_size=1, latch=True)

    rospy.logwarn("Emergency keyboard node started.")
    rospy.logwarn("Press SPACE to stop motors. Press q to quit this node.")

    old_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
            key = sys.stdin.read(1)

            if key == " ":
                rospy.logfatal("EMERGENCY STOP KEY PRESSED")
                pub.publish(Bool(data=True))

            elif key == "q":
                break

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    main()