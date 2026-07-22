#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This script initializes all CAN interfaces and sends a stop command to all motors
Author : Florent Pralong
Date of creation : 05/06/2026
Version : 1.0
"""

# ---------------------------
# IMPORTS
# ---------------------------

import os
import time
import myactuator_rmd_py as rmd


# ---------------------------
# FUNCTIONS
# ---------------------------

def init_canports():

    'Bring up network devices for each CAN channel at bitrate 1 Mbps.' 

    for can in ["can0", "can1", "can2", "can3"]:
        os.system(f"sudo ip link set {can} down")
        time.sleep(0.1)
        os.system(f"sudo ip link set {can} up type can bitrate 1000000")
        time.sleep(0.1)


# ---------------------------
# MAIN PROGRAM
# ---------------------------

# Setup CAN channels and motors.
init_canports()
motor = {}

# Front right leg.
can_fr = rmd.CanDriver("can0")
fr_shoulder = motor[1] = rmd.ActuatorInterface(can_fr, 1)
fr_hip = motor[3] = rmd.ActuatorInterface(can_fr, 3)
fr_knee = motor[5] = rmd.ActuatorInterface(can_fr, 5)
print("Front right leg motor ID's set.")

# Front left leg.
can_fl = rmd.CanDriver("can1")
fl_shoulder = motor[2] = rmd.ActuatorInterface(can_fl, 2)
fl_hip = motor[4] = rmd.ActuatorInterface(can_fl, 4)
fl_knee = motor[6] = rmd.ActuatorInterface(can_fl, 6)
print("Front left leg motor ID's set.")

# Hind right leg.
can_hr = rmd.CanDriver("can2")
hr_shoulder = motor[11] = rmd.ActuatorInterface(can_hr, 11)
hr_hip = motor[13] = rmd.ActuatorInterface(can_hr, 13)
hr_knee = motor[15] = rmd.ActuatorInterface(can_hr, 15)
hr_ankle = motor[17] = rmd.ActuatorInterface(can_hr, 17)
print("Hind right leg motor ID's set.")

# Hind left leg.
can_hl = rmd.CanDriver("can3")
hl_shoulder = motor[10] = rmd.ActuatorInterface(can_hl, 10)
hl_hip = motor[12] = rmd.ActuatorInterface(can_hl, 12)
hl_knee = motor[14] = rmd.ActuatorInterface(can_hl, 14)
hl_ankle = motor[16] = rmd.ActuatorInterface(can_hl, 16)
print("Hind left leg motor ID's set.")


#Create list of motor IDs and send stop command to all motors.
motor_IDs = list(range(1, 7)) + list(range(10, 18))

for id in motor_IDs:
    motor[id].stopMotor()