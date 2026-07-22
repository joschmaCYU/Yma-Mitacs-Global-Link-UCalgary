#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import os


class PolicyConfig:
    def __init__(self):
        # ---- IMU & Commands ----
        self.flip_gravity_sign = bool(rospy.get_param("~flip_gravity_sign", False))
        self.imu_use_R_transpose = bool(rospy.get_param("~imu_use_R_transpose", True))
        self.swap_cmd_xy = bool(rospy.get_param("~swap_cmd_xy", False))
        self.invert_cmd_vx = bool(rospy.get_param("~invert_cmd_vx", False))
        self.invert_cmd_vy = bool(rospy.get_param("~invert_cmd_vy", False))
        self.obs_center_q0 = bool(rospy.get_param("~obs_center_q0", True))

        # ---- Basics & IO ----
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.output_topic = rospy.get_param("~output_topic", "/joint_targets_rl")
        self.use_cmd_vel = bool(rospy.get_param("~use_cmd_vel", True))
        self.use_odom = bool(rospy.get_param("~use_odom", False))
        self.use_gz_links = bool(rospy.get_param("~use_gazebo_link_states", True))
        self.base_link_name = rospy.get_param("~base_link_name", "base")
        self.x_max_m = float(rospy.get_param("~x_max_m", 0.5))
        self.y_max_m = float(rospy.get_param("~y_max_m", 0.5))
        self.episode_len_s = float(rospy.get_param("~episode_len_s", 6.0))

        # ---- Autonomous Mode ----
        self.autonomous_mode = bool(rospy.get_param("~autonomous_mode", True))
        self.auto_cmd_type = (
            rospy.get_param("~auto_cmd_type", "constant").strip().lower()
        )
        self.auto_x_m = float(rospy.get_param("~auto_x_m", 0.2))
        self.auto_y_m = float(rospy.get_param("~auto_y_m", 0.2))
        self.auto_wz = float(rospy.get_param("~auto_wz", 0.2))
        self.auto_cycle_s = float(rospy.get_param("~auto_cycle_s", 6.0))

        # ---- Safety & Filters ----
        self.clamp_deg = float(rospy.get_param("~clamp_deg", 25.0))
        self.smooth_alpha = float(rospy.get_param("~smooth_alpha", 0.25))
        self.action_gain = float(rospy.get_param("~action_gain", 1.0))
        self.cmd_vel_threshold = float(rospy.get_param("~cmd_vel_threshold", 0.05))
        self.scale_by_cmd = bool(rospy.get_param("~scale_by_cmd", True))
        self.yaw_gain_in_norm = float(rospy.get_param("~yaw_gain_in_norm", 0.5))
        self.hold_mode = rospy.get_param("~hold_mode", "neutral").strip().lower()

        # ---- Kinematics ----
        self.joint_order = rospy.get_param(
            "~joint_order",
            [
                "FL_HAA",
                "FL_HFE",
                "FL_KFE",
                "FR_HAA",
                "FR_HFE",
                "FR_KFE",
                "HL_HAA",
                "HL_HFE",
                "HL_KFE",
                "HL_AFE",
                "HR_HAA",
                "HR_HFE",
                "HR_KFE",
                "HR_AFE",
            ],
        )

        self.joint_order_obs = rospy.get_param(
            "~joint_order_obs",
            [
                "FL_HAA",
                "FR_HAA",
                "HL_HAA",
                "HR_HAA",
                "FL_HFE",
                "FR_HFE",
                "HL_HFE",
                "HR_HFE",
                "FL_KFE",
                "FR_KFE",
                "HL_KFE",
                "HR_KFE",
                "HL_AFE",
                "HR_AFE",
            ],
        )

        self.neutral_positions = rospy.get_param(
            "~neutral_positions",
            {
                "FL_HAA": 0.0,
                "FL_HFE": 0.4102,
                "FL_KFE": -1.2716,
                "FR_HAA": 0.0,
                "FR_HFE": 0.4102,
                "FR_KFE": -1.2716,
                "HL_HAA": 0.0,
                "HL_HFE": -0.6981,
                "HL_KFE": 1.676,
                "HL_AFE": -1.7219,
                "HR_HAA": 0.0,
                "HR_HFE": -0.6981,
                "HR_KFE": 1.676,
                "HR_AFE": -1.7219,
            },
        )

        self.model_q0 = rospy.get_param(
            "~model_q0",
            {
                "FL_HAA": 0.0,
                "FR_HAA": 0.0,
                "HL_HAA": 0.0,
                "HR_HAA": 0.0,
                "FL_HFE": 0.4102,
                "FR_HFE": 0.4102,
                "HL_HFE": -0.6981,
                "HR_HFE": -0.6981,
                "FL_KFE": -1.2716,
                "FR_KFE": -1.2716,
                "HL_KFE": 1.676,
                "HR_KFE": 1.676,
                "HL_AFE": -1.7219,
                "HR_AFE": -1.7219,
            },
        )

        # ---- Profiles & Overrides ----
        self.policy_profile = rospy.get_param("~policy_profile", None)
        self.profiles = rospy.get_param("~policy_profiles", {})
        self.prof = (
            self.profiles.get(self.policy_profile, {}) if self.policy_profile else {}
        )
        self.profile_name = self.policy_profile or "default"

        self.model_format = self.prof.get(
            "model_format", rospy.get_param("~model_format", "onnx").strip().lower()
        )
        self.model_path_par = self.prof.get(
            "model_path", rospy.get_param("~model_path", "")
        )
        self.ort_input_name_override = self.prof.get("ort_input_name", None)

        limits = self.prof.get("limits", {})
        if limits:
            self.x_max_m = float(limits.get("x_max_m", self.x_max_m))
            self.y_max_m = float(limits.get("y_max_m", self.y_max_m))

        self.action_order_model = self.prof.get(
            "action_order_model",
            rospy.get_param("~action_order_model", list(self.joint_order_obs)),
        )
        self.action_mode = self.prof.get(
            "action_mode", rospy.get_param("~action_mode", "delta").strip().lower()
        )
        self.base_for_delta = self.prof.get(
            "base_for_delta",
            rospy.get_param("~base_for_delta", "model_q0").strip().lower(),
        )

        if "model_q0" in self.prof:
            self.model_q0.update(self.prof["model_q0"])

        self.obs_fields = self.prof.get(
            "obs_fields",
            [
                "base_lin_vel",
                "base_ang_vel",
                "gravity",
                "pose_xy",
                "zeros2",
                "q14",
                "dq14",
                "last_actions14",
                "height_scan",
                "time_remaining",
            ],
        )

        self.model_path = self._resolve_model_path(self.model_path_par)

    def _resolve_model_path(self, path_param: str) -> str:
        if not path_param:
            pkg = rospkg.RosPack().get_path("quadruped_control")
            return os.path.join(pkg, "policies", "policy.onnx")
        if os.path.isabs(path_param):
            return path_param
        pkg = rospkg.RosPack().get_path("quadruped_control")
        return os.path.join(pkg, path_param)
