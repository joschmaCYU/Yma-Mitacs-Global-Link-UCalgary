#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist


def quat_to_rot(qw, qx, qy, qz):
    R = np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ],
        dtype=np.float32,
    )
    return R


class ObservationManager:
    def __init__(self, config):
        self.cfg = config
        self.latest_js = None
        self.latest_imu = None
        self.latest_cmd = Twist()
        self.latest_odom = None
        self.latest_gz = None
        self.latest_scan = None
        self.ep_start = rospy.Time.now()

    # ---- Callbacks ----
    def cb_joint_states(self, msg):
        self.latest_js = msg

    def cb_imu(self, msg):
        self.latest_imu = msg

    def cb_cmd_vel(self, msg):
        self.latest_cmd = msg

    def cb_odom(self, msg):
        self.latest_odom = msg

    def cb_gz(self, msg):
        self.latest_gz = msg

    def cb_lidar(self, msg):
        self.latest_scan = msg

    # ---- Data Extraction ----
    def get_base_lin_vel_body(self):
        # if self.cfg.use_odom and self.latest_odom is not None:
        #     v = self.latest_odom.twist.twist.linear
        #     return np.array([v.x, v.y, v.z], dtype=np.float32)

        if (
            self.cfg.use_gz_links
            and self.latest_gz is not None
            and self.latest_imu is not None
        ):
            names = self.latest_gz.name
            idx = next(
                (
                    i
                    for i, n in enumerate(names)
                    if n.endswith("::" + self.cfg.base_link_name)
                    or n == self.cfg.base_link_name
                ),
                -1,
            )
            if idx < 0:
                idx = next(
                    (
                        i
                        for i, n in enumerate(names)
                        if n.endswith("::base") or n == "base"
                    ),
                    -1,
                )

            if idx >= 0:
                vw = self.latest_gz.twist[idx].linear
                v_world = np.array([vw.x, vw.y, vw.z], dtype=np.float32)
                q = self.latest_imu.orientation
                R = quat_to_rot(q.w, q.x, q.y, q.z)
                v_body = (
                    R.T.dot(v_world)
                    if self.cfg.imu_use_R_transpose
                    else R.dot(v_world)
                )
                return v_body.astype(np.float32)
        return np.zeros(3, dtype=np.float32)

    def get_base_ang_vel_body_or_zero(self):
        if self.latest_imu is None:
            return np.zeros(3, np.float32)
        w = self.latest_imu.angular_velocity
        return np.array([w.x, w.y, w.z], dtype=np.float32)

    def get_projected_gravity_or_default(self):
        if self.latest_imu is None:
            return np.array([0.0, 0.0, -1.0], np.float32)
        q = self.latest_imu.orientation
        R = quat_to_rot(q.w, q.x, q.y, q.z)
        # g = R.dot(np.array([0, 0, -1], dtype=np.float32))
        # CORRECTION : Utilisation de R.T (Transposée) pour passer du World au Body Frame
        g = R.T.dot(np.array([0, 0, -1], dtype=np.float32))
        return -g if self.cfg.flip_gravity_sign else g

    def _get_pose_cmd_norm(self):
        vx = (
            float(getattr(self.latest_cmd.linear, "x", 0.0)) if self.latest_cmd else 0.0
        )
        vy = (
            float(getattr(self.latest_cmd.linear, "y", 0.0)) if self.latest_cmd else 0.0
        )
        if self.cfg.swap_cmd_xy:
            vx, vy = vy, vx
        if self.cfg.invert_cmd_vx:
            vx = -vx
        if self.cfg.invert_cmd_vy:
            vy = -vy
        arr = np.array(
            [vx / max(self.cfg.x_max_m, 1e-6), vy / max(self.cfg.y_max_m, 1e-6)],
            dtype=np.float32,
        )
        return np.clip(arr, -1.0, 1.0)

    def get_q_dq_obs(self):
        q_list, dq_list = [], []
        if self.latest_js is None:
            return np.zeros(14, np.float32), np.zeros(14, np.float32)

        idx = {n: i for i, n in enumerate(self.latest_js.name)}
        for jn in self.cfg.joint_order_obs:
            if jn in idx:
                i = idx[jn]
                q_val = (
                    self.latest_js.position[i]
                    if len(self.latest_js.position) > i
                    else 0.0
                )
                dq_val = (
                    self.latest_js.velocity[i]
                    if len(self.latest_js.velocity) > i
                    else 0.0
                )
            else:
                q_val, dq_val = 0.0, 0.0

            # if self.cfg.obs_center_q0:
            #     q_val -= float(self.cfg.model_q0.get(jn, 0.0))
            q_list.append(q_val)
            dq_list.append(dq_val)

        return np.array(q_list, np.float32), np.array(dq_list, np.float32)

    def get_height_scan(self):
        target_dim = 187

        # Si aucun message n'est reçu, on simule un sol plat (zéros)
        if self.latest_scan is None or len(self.latest_scan.data) == 0:
            return np.zeros(target_dim, dtype=np.float32)

        # On lit le tableau de Gazebo/ROS
        data = np.array(self.latest_scan.data, dtype=np.float32)

        # Sécurité : on s'assure qu'il fait EXACTEMENT 187 de long
        if len(data) == target_dim:
            return data
        else:
            res = np.zeros(target_dim, dtype=np.float32)
            l = min(len(data), target_dim)
            res[:l] = data[:l]
            return res

    def get_time_remaining(self):
        elapsed = (rospy.Time.now() - self.ep_start).to_sec()
        T = max(self.cfg.episode_len_s, 0.1)
        return max(0.0, T - (elapsed % T))

    def build_obs_vector(self, last_actions_model):
        raw_lin_vel = self.get_base_lin_vel_body()
        raw_ang_vel = self.get_base_ang_vel_body_or_zero()
        raw_q14, raw_dq14 = self.get_q_dq_obs()
        raw_height = self.get_height_scan()

        bank = {
            # Application des échelles (Scales)
            "base_lin_vel": raw_lin_vel,
            "base_ang_vel": raw_ang_vel,
            "gravity": self.get_projected_gravity_or_default(),
            "pose_xy": self._get_pose_cmd_norm(),
            "zeros2": np.zeros(2, np.float32),
            "q14": raw_q14,
            "dq14": raw_dq14,
            "last_actions14": np.array(last_actions_model, np.float32),
            "time_remaining": np.array([self.get_time_remaining()], np.float32),
            "height_scan": raw_height,
        }

        parts = []
        for key in self.cfg.obs_fields:
            if key not in bank:
                parts.append(np.zeros(1, np.float32))
            else:
                parts.append(bank[key])

        obs_np = np.concatenate(parts, axis=0).astype(np.float32)

        expected_dim = 243
        current_dim = obs_np.shape[0]

        if current_dim < expected_dim:
            padding = np.zeros(expected_dim - current_dim, dtype=np.float32)
            obs_np = np.concatenate([obs_np, padding])
        elif current_dim > expected_dim:
            obs_np = obs_np[:expected_dim]

        return obs_np
