#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import traceback
import rospy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates

import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config import PolicyConfig
from observation import ObservationManager

import onnxruntime as ort


class PolicyNodeTorch:
    def __init__(self):
        rospy.init_node("policy_node_torch")

        self.cfg = PolicyConfig()
        self.obs_mgr = ObservationManager(self.cfg)

        self.num_ctrl_joints = len(self.cfg.joint_order)
        self.last_action_ctrl = np.zeros(self.num_ctrl_joints, dtype=np.float32)
        self.filtered_action = np.zeros(self.num_ctrl_joints, dtype=np.float32)
        self.last_action_model = np.zeros(
            len(self.cfg.action_order_model), dtype=np.float32
        )

        # ---- IO ROS ----
        rospy.Subscriber(
            "/joint_states", JointState, self.obs_mgr.cb_joint_states, queue_size=50
        )
        rospy.Subscriber("/imu", Imu, self.obs_mgr.cb_imu, queue_size=50)
        rospy.Subscriber(
            "/lidar", Float32MultiArray, self.obs_mgr.cb_lidar, queue_size=20
        )

        if self.cfg.use_odom:
            rospy.Subscriber("/odom", Odometry, self.obs_mgr.cb_odom, queue_size=20)
        if self.cfg.use_gz_links:
            rospy.Subscriber(
                "/gazebo/link_states", LinkStates, self.obs_mgr.cb_gz, queue_size=20
            )

        self.pub = rospy.Publisher(self.cfg.output_topic, JointState, queue_size=10)

        # ---- Load Model ----
        self._ort_session = None
        self._ort_input_name = None
        self._load_model()

        # ---- Start Loop ----
        self._t0 = rospy.Time.now().to_sec()
        self.loop()

    def _load_model(self):
        try:
            self._ort_session = ort.InferenceSession(
                self.cfg.model_path, providers=["CPUExecutionProvider"]
            )
            if self.cfg.ort_input_name_override:
                self._ort_input_name = self.cfg.ort_input_name_override
            else:
                inputs = self._ort_session.get_inputs()
                if not inputs:
                    raise RuntimeError("ONNX model has no inputs.")
                self._ort_input_name = inputs[0].name
            rospy.loginfo(
                f"✅ ONNX loaded: {self.cfg.model_path} (input='{self._ort_input_name}')"
            )
        except Exception:
            rospy.logerr(f"❌ Error loading ONNX:\n{traceback.format_exc()}")

    def run_policy(self, obs_np):
        if self._ort_session and self._ort_input_name:
            try:
                outputs = self._ort_session.run(
                    None, {self._ort_input_name: obs_np.reshape(1, -1)}
                )
                return np.asarray(outputs[0]).reshape(-1).astype(np.float32)
            except Exception as e:
                rospy.logerr_throttle(2.0, f"Error ONNX: {repr(e)}")

        # Dummy fallback
        t = rospy.get_time()
        m = len(self.cfg.action_order_model)
        return (
            0.2 * np.sin(2.0 * math.pi * 0.3 * t + np.linspace(0, 2 * math.pi, m))
        ).astype(np.float32)

    def map_model_action_to_control(self, a_model):
        a_model = np.asarray(a_model, dtype=np.float32).reshape(-1)
        M = len(self.cfg.action_order_model)
        if a_model.shape[0] != M:
            if a_model.shape[0] > M:
                a_model = a_model[:M]
            else:
                a_model = np.concatenate(
                    [a_model, np.zeros(M - len(a_model), np.float32)]
                )

        ctrl = np.zeros(self.num_ctrl_joints, dtype=np.float32)
        mp = {n: i for i, n in enumerate(self.cfg.action_order_model)}
        for ci, cname in enumerate(self.cfg.joint_order):
            ctrl[ci] = a_model[mp[cname]] if cname in mp else 0.0
        return ctrl

    def clamp_and_smooth(self, q_target):
        if np.any(~np.isfinite(q_target)):
            return self.filtered_action.copy()

        rad = math.radians(self.cfg.clamp_deg)
        q_target = np.clip(q_target, -10 * rad, 10 * rad)
        alpha = float(np.clip(self.cfg.smooth_alpha, 0.0, 1.0))
        self.filtered_action = alpha * q_target + (1.0 - alpha) * self.filtered_action
        return self.filtered_action.copy()

    def build_base_vector(self):
        pose = (
            self.cfg.model_q0
            if self.cfg.base_for_delta == "model_q0"
            else self.cfg.neutral_positions
        )
        q_base = np.zeros(self.num_ctrl_joints, dtype=np.float32)
        for i, jn in enumerate(self.cfg.joint_order):
            q_base[i] = float(pose.get(jn, 0.0))
        return q_base

    def loop(self):
        rate = rospy.Rate(self.cfg.rate_hz)
        q_base = self.build_base_vector()

        while not rospy.is_shutdown():
            if self.cfg.autonomous_mode:
                t = rospy.Time.now().to_sec() - self._t0
                vx, vy, wz = self.cfg.auto_x_m, self.cfg.auto_y_m, self.cfg.auto_wz
                if self.cfg.auto_cmd_type == "sine":
                    phase = 2.0 * math.pi * (t / max(self.cfg.auto_cycle_s, 1e-6))
                    vx *= math.sin(phase)
                    vy *= math.sin(phase)
                    wz *= math.sin(phase)

                auto_twist = Twist()
                auto_twist.linear.x, auto_twist.linear.y, auto_twist.angular.z = (
                    vx,
                    vy,
                    wz,
                )
                self.obs_mgr.latest_cmd = auto_twist

            obs = self.obs_mgr.build_obs_vector(self.last_action_model)
            if obs is None:
                rate.sleep()
                continue

            # --- Deadman / Safety Stop ---
            latest_cmd = self.obs_mgr.latest_cmd
            vx = float(getattr(latest_cmd.linear, "x", 0.0)) if latest_cmd else 0.0
            vy = float(getattr(latest_cmd.linear, "y", 0.0)) if latest_cmd else 0.0
            wz = float(getattr(latest_cmd.angular, "z", 0.0)) if latest_cmd else 0.0
            cmd_mag = math.sqrt(vx**2 + vy**2 + (self.cfg.yaw_gain_in_norm * wz) ** 2)

            if (
                not self.cfg.autonomous_mode
                and self.cfg.use_cmd_vel
                and cmd_mag < self.cfg.cmd_vel_threshold
            ):
                out = (
                    q_base.copy()
                    if self.cfg.hold_mode == "neutral"
                    else np.zeros(self.num_ctrl_joints, dtype=np.float32)
                )
                self.last_action_ctrl = self.filtered_action = out.copy()

                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = list(self.cfg.joint_order)
                msg.position = out.tolist()
                self.pub.publish(msg)

                rate.sleep()
                continue

            # --- Model Inference ---
            a_model = self.run_policy(obs)
            self.last_action_model = np.asarray(a_model, dtype=np.float32).copy()
            a_ctrl = self.map_model_action_to_control(a_model)

            if self.cfg.scale_by_cmd:
                pose_xy = self.obs_mgr._get_pose_cmd_norm()
                scale = max(
                    0.0, min(float(math.sqrt(pose_xy[0] ** 2 + pose_xy[1] ** 2)), 1.0)
                )
                a_ctrl *= scale

            if self.cfg.action_mode == "delta":
                q_cmd = q_base + (self.cfg.action_gain * a_ctrl)
            else:
                q_cmd = self.cfg.action_gain * a_ctrl

            q_cmd = self.clamp_and_smooth(q_cmd)
            self.last_action_ctrl = q_cmd.copy()

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = list(self.cfg.joint_order)
            msg.position = q_cmd.tolist()
            self.pub.publish(msg)

            rate.sleep()


if __name__ == "__main__":
    try:
        PolicyNodeTorch()
    except rospy.ROSInterruptException:
        pass
