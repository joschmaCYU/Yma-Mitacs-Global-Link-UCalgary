#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import rospy
import rospkg

from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32MultiArray

class QuadrupedFakeObsDatasNode:

    def __init__(self):
        rospy.init_node("quadruped_fake_obs_datas_node")

        self.csv_name = rospy.get_param("~csv_name", "joint_positions_flat.csv")
        self.publish_hz = rospy.get_param("~publish_hz", 50.0)
        self.loop = rospy.get_param("~loop", True)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("f_quadruped_control")

        self.csv_path = rospy.get_param(
            "~csv_path", "/root/catkin_ws/src/mitacs/joschka/data/joint_positions_flat.csv"
        )

        self.pub_imu = rospy.Publisher("/imu", Imu, queue_size=10)
        self.pub_cmd = rospy.Publisher("/cmd", Float32MultiArray, queue_size=10)
        self.pub_joint_states = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.pub_lidar = rospy.Publisher("/lidar", LaserScan, queue_size=10)

        # --- NOUVEAU : PUBLISHER DES CIBLES CSV POUR LE REPLAY ---
        self.pub_csv_targets = rospy.Publisher("/joint_targets_csv", JointState, queue_size=10)

        self.joint_names = [
            "FL_HAA", "FR_HAA", "HL_HAA", "HR_HAA",
            "FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE",
            "FL_KFE", "FR_KFE", "HL_KFE", "HR_KFE",
            "HL_AFE", "HR_AFE"
        ]

        self.data = self.load_csv()

        rospy.loginfo("Loaded %d rows from %s", len(self.data), self.csv_path)
        rospy.loginfo("Publishing fake observation data at %.2f Hz", self.publish_hz)

    def get_float(self, row, key, line_index):
        try:
            return float(row[key])
        except KeyError:
            rospy.logerr("Missing column '%s' in CSV at line %d", key, line_index + 2)
            raise
        except ValueError:
            rospy.logerr("Invalid float for column '%s' in CSV at line %d", key, line_index + 2)
            raise

    # Sécurité au cas où la colonne target_ n'existe pas
    def get_target_safe(self, row, key):
        try:
            return float(row[key])
        except:
            return 0.0

    def load_csv(self):
        rows = []
        with open(self.csv_path, "r") as csv_file:
            reader = csv.DictReader(csv_file)

            for line_index, row in enumerate(reader):
                imu_data = {
                    "base_lin_vel": [self.get_float(row, "obs_0", line_index), self.get_float(row, "obs_1", line_index), self.get_float(row, "obs_2", line_index)],
                    "base_ang_vel": [self.get_float(row, "obs_3", line_index), self.get_float(row, "obs_4", line_index), self.get_float(row, "obs_5", line_index)],
                    "projected_gravity": [self.get_float(row, "obs_6", line_index), self.get_float(row, "obs_7", line_index), self.get_float(row, "obs_8", line_index)],
                }

                cmd_data = [self.get_float(row, "obs_9", line_index), self.get_float(row, "obs_10", line_index), self.get_float(row, "obs_11", line_index), self.get_float(row, "obs_12", line_index)]

                joint_pos = [self.get_float(row, f"obs_{i}", line_index) for i in range(13, 27)]
                joint_vel = [self.get_float(row, f"obs_{i}", line_index) for i in range(27, 41)]
                lidar_data = [self.get_float(row, f"obs_{i}", line_index) for i in range(41, 55)]

                # --- NOUVEAU : EXTRACTION DES CIBLES EXACTES ---
                csv_targets = [self.get_target_safe(row, "target_" + j) for j in self.joint_names]

                rows.append({
                    "imu": imu_data,
                    "cmd": cmd_data,
                    "joint_pos": joint_pos,
                    "joint_vel": joint_vel,
                    "lidar": lidar_data,
                    "csv_targets": csv_targets,
                })

        if not rows:
            raise RuntimeError("CSV file is empty")
        return rows

    def build_imu_msg(self, data, stamp):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.linear_acceleration.x = data["base_lin_vel"][0]
        msg.linear_acceleration.y = data["base_lin_vel"][1]
        msg.linear_acceleration.z = data["base_lin_vel"][2]
        msg.angular_velocity.x = data["base_ang_vel"][0]
        msg.angular_velocity.y = data["base_ang_vel"][1]
        msg.angular_velocity.z = data["base_ang_vel"][2]
        msg.orientation.x = data["projected_gravity"][0]
        msg.orientation.y = data["projected_gravity"][1]
        msg.orientation.z = data["projected_gravity"][2]
        msg.orientation.w = 0.0
        return msg

    def build_cmd_msg(self, data):
        msg = Float32MultiArray()
        msg.data = data
        return msg

    def build_joint_state_msg(self, positions, velocities, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = velocities
        return msg

    def build_lidar_msg(self, data, stamp):
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "lidar"
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = (msg.angle_max - msg.angle_min) / max(len(data) - 1, 1)
        msg.scan_time = 1.0 / self.publish_hz
        msg.range_max = 10.0
        msg.ranges = data
        return msg

    def publish_row(self, row):
        stamp = rospy.Time.now()

        self.pub_imu.publish(self.build_imu_msg(row["imu"], stamp))
        self.pub_cmd.publish(self.build_cmd_msg(row["cmd"]))
        self.pub_joint_states.publish(self.build_joint_state_msg(row["joint_pos"], row["joint_vel"], stamp))
        self.pub_lidar.publish(self.build_lidar_msg(row["lidar"], stamp))

        # --- NOUVEAU : PUBLICATION DES CIBLES ---
        msg_targets = JointState()
        msg_targets.header.stamp = stamp
        msg_targets.name = self.joint_names
        msg_targets.position = row["csv_targets"]
        self.pub_csv_targets.publish(msg_targets)

    def run(self):
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown():
            for row in self.data:
                if rospy.is_shutdown():
                    break
                self.publish_row(row)
                rate.sleep()
            if not self.loop:
                break

if __name__ == "__main__":
    try:
        node = QuadrupedFakeObsDatasNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
