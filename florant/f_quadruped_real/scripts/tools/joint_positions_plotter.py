#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node is used for development task, it can plot values by reading datas on specifics nodes.
Author : Florent Pralong
Date of creation : 16/06/2026
Version : 1.0
"""
# ---------------------------
# IMPORTS
# ---------------------------

import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from collections import defaultdict
from std_msgs.msg import Bool


class JointPositionPlotter:

    # ---------------------------
    # INITIALIZATION
    # ---------------------------
    def __init__(self):
        rospy.init_node("joint_position_plotter", anonymous=True)

        self.start_time = rospy.Time.now()
        self.plot_frozen = False

        self.measured_time = defaultdict(list)
        self.measured_position = defaultdict(list)

        self.target_time = defaultdict(list)
        self.target_position = defaultdict(list)

        self.max_points = rospy.get_param("~max_points", 5000)
        self.plot_rate = rospy.get_param("~plot_rate", 10.0)

        #-----------------------------------------
        # Direction of rotation inversion of specifics motors

        self.joints_to_plot = rospy.get_param(
            "~joints_to_plot",
            ["FL_HAA", "FL_HFE", "FL_KFE"]
        )

        self.inverted_joints = {
            "FL_HAA", "FL_HFE", "FL_KFE"
        }

        #-------------------------------------------
        
        # Subscribers to /joint_states /joint_targets_rl and /plot_frozen

        rospy.Subscriber(
            "/joint_states",
            JointState,
            self.joint_states_callback,
            queue_size=100
        )

        rospy.Subscriber(
            "/joint_targets_rl",
            JointState,
            self.joint_targets_callback,
            queue_size=100
        )

        rospy.Subscriber(
            "/plot_frozen",
            Bool,
            self.plot_frozen_callback,
            queue_size=1
        )

        # Plot configuration

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("Time since node start [s]")
        self.ax.set_ylabel("Joint position [rad]")
        self.ax.set_title("Measured vs Target Joint Positions")
        self.ax.grid(True)

        self.joint_colors = {
            "FL_HAA": "tab:blue",
            "FL_HFE": "tab:orange",
            "FL_KFE": "tab:green",

            "FR_HAA": "tab:red",
            "FR_HFE": "tab:purple",
            "FR_KFE": "tab:brown",

            "HL_HAA": "tab:pink",
            "HL_HFE": "tab:gray",
            "HL_KFE": "tab:olive",
            "HL_AFE": "tab:cyan",

            "HR_HAA": "black",
            "HR_HFE": "darkblue",
            "HR_KFE": "darkred",
            "HR_AFE": "darkgreen",
        }

    # ---------------------------
    # INITIALIZATION
    # ---------------------------
   
    # Time managment
    def get_node_time(self):
        return (rospy.Time.now() - self.start_time).to_sec()
    
    # Used to freeze the plot with a shortcut
    def plot_frozen_callback(self, msg):
        self.plot_frozen = msg.data

        if self.plot_frozen:
            rospy.loginfo("Plot frozen")
        else:
            rospy.loginfo("Plot resumed")
    
    # Store datas from the nodes
    def store_data(self, msg, time_dict, position_dict, invert=False):
        t = self.get_node_time()

        for name, position in zip(msg.name, msg.position):

            if invert and name in self.inverted_joints:
                position = -position

            time_dict[name].append(t)
            position_dict[name].append(position)

            if len(time_dict[name]) > self.max_points:
                time_dict[name].pop(0)
                position_dict[name].pop(0)

    # Read the /joint_states nodes
    def joint_states_callback(self, msg):
        self.store_data(
            msg,
            self.measured_time,
            self.measured_position,
            invert=True
        )

    # Read the /joint_targets
    def joint_targets_callback(self, msg):
        self.store_data(
            msg,
            self.target_time,
            self.target_position,
            invert=False
        )

    # Plot update function
    def update_plot(self):
        
        if self.plot_frozen:
            plt.pause(0.001)
            return
        
        self.ax.clear()
        
        self.ax.set_xlabel("Time since node start [s]",fontsize = 24)
        self.ax.set_ylabel("Joint position [rad]",fontsize = 24)
        self.ax.set_title("Measured vs Target Joint Positions",fontsize = 50)
        self.ax.tick_params(axis='both', labelsize=24)
        self.ax.grid(True)

        joint_names = self.joints_to_plot

        for joint in joint_names:
            color = self.joint_colors.get(joint, None)

            if joint in self.measured_position:
                self.ax.plot(
                    self.measured_time[joint],
                    self.measured_position[joint],
                    label=f"{joint} measured",
                    linestyle="-",
                    marker="o",
                    markersize=3,
                    color = color
                )

            if joint in self.target_position:
                self.ax.plot(
                    self.target_time[joint],
                    self.target_position[joint],
                    label=f"{joint} target",
                    linestyle="--",
                    marker="x",
                    markersize=4,
                    color = color
                )

        self.ax.legend(loc="upper right", fontsize=20, ncol=2)
        plt.pause(0.001)

    # Run
    def run(self):
        rate = rospy.Rate(self.plot_rate)

        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()

# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    try:
        node = JointPositionPlotter()
        node.run()
    except rospy.ROSInterruptException:
        pass
