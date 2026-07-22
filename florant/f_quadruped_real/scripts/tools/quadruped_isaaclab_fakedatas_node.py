#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node controls one leg of the quadruped in real hardware, by subscribing to joint target commands and sending position setpoints to the motors via CAN bus.
Author : Florent Pralong
Date of creation : 10/06/2026
Version : 1.0
"""

# ---------------------------
# IMPORTS
# ---------------------------

import csv
import rospy
import rospkg
from sensor_msgs.msg import JointState

class IsaacLabFakeDatasNode:

    # ---------------------------
    # INITIALIZATION
    # ---------------------------

    def __init__(self):
        rospy.init_node("quadruped_isaaclab_fakedatas_node")

        # ---------------------------
        # PARAMETERS
        # ---------------------------

        self.topic_name = rospy.get_param("~topic_name", "/joint_targets_rl")
        self.csv_name = rospy.get_param("~csv_name", "joint_positions_flat_video2_cut.csv")
        self.publish_hz = rospy.get_param("~publish_hz", 50.0)
        self.loop = rospy.get_param("~loop", False)
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("f_quadruped_real")
        self.csv_path = rospy.get_param(
            "~csv_path",
            package_path + "/src/" + self.csv_name
        )

        # Publisher
        self.publisher = rospy.Publisher(
            self.topic_name,
            JointState,
            queue_size=10
        )

        # Joints configuration
        self.output_joint_names = [
            "FL_HAA", "FL_HFE", "FL_KFE",
            "FR_HAA", "FR_HFE", "FR_KFE",
            "HL_HAA", "HL_HFE", "HL_KFE", "HL_AFE",
            "HR_HAA", "HR_HFE", "HR_KFE", "HR_AFE"
        ]

        self.data = self.load_csv()

        rospy.loginfo("Loaded %d rows from %s", len(self.data), self.csv_path)
        rospy.loginfo("Publishing JointState on %s at %.2f Hz", self.topic_name, self.publish_hz)

    # Load the csv with the positions list to emulate the control pollicy
    def load_csv(self):
        rows = []

        with open(self.csv_path, "r") as csv_file:
            reader = csv.DictReader(csv_file)

            for line_index, row in enumerate(reader):
                try:
                    positions = [
                        float(row["FL_HAA"]),
                        float(row["FL_HFE"]),
                        float(row["FL_KFE"]),

                        float(row["FR_HAA"]),
                        float(row["FR_HFE"]),
                        float(row["FR_KFE"]),

                        float(row["HL_HAA"]),
                        float(row["HL_HFE"]),
                        float(row["HL_KFE"]),
                        float(row["HL_AFE"]),

                        float(row["HR_HAA"]),
                        float(row["HR_HFE"]),
                        float(row["HR_KFE"]),
                        float(row["HR_AFE"]),
                    ]

                    rows.append(positions)

                except KeyError as e:
                    rospy.logerr("Missing column in CSV at line %d: %s", line_index + 2, str(e))
                    raise

                except ValueError as e:
                    rospy.logerr("Invalid float in CSV at line %d: %s", line_index + 2, str(e))
                    raise

        if not rows:
            raise RuntimeError("CSV file is empty")

        return rows

    # Run and publish positions to /joint_targets_rl
    def run(self):
        rate = rospy.Rate(self.publish_hz)

        while not rospy.is_shutdown():
            for positions in self.data:
                if rospy.is_shutdown():
                    break

                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = self.output_joint_names
                msg.position = positions
                msg.velocity = []
                msg.effort = []

                self.publisher.publish(msg)
                rate.sleep()

            if not self.loop:
                rospy.loginfo("Finished publishing Isaac Lab fake data sequence")
                break


# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    try:
        node = IsaacLabFakeDatasNode()
        node.run()

    except rospy.ROSInterruptException:
        pass

    except Exception as e:
        rospy.logerr("Node failed: %s", str(e))