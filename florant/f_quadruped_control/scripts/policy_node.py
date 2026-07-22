#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node runs the control policy (Pure ROS Subscriber Mode)
Author : Florent Pralong
Date of creation : 28/06/2026
Version : 1.0 (Restored Original)
"""

# ---------------------------
# IMPORTS
# ---------------------------

import os
import numpy as np
import rospy
import rospkg
import csv
import yaml
import threading

from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

try:
    import onnxruntime as ort
except Exception:
    ort = None

try:
    import torch
    torch.set_grad_enabled(False)
except Exception:
    torch = None

# ---------------------------
# Utility functions
# ---------------------------

def quat_to_rot(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - qz*qw),     2 * (qx*qz + qy*qw)],
        [2 * (qx*qy + qz*qw),     1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz - qx*qw)],
        [2 * (qx*qz - qy*qw),     2 * (qy*qz + qx*qw),     1 - 2 * (qx*qx + qy*qy)]
    ], dtype=np.float32)

def resolve_model_path(path_param):
    if path_param:
        if os.path.isabs(path_param):
            return path_param
        pkg = rospkg.RosPack().get_path("f_quadruped_control")
        return os.path.join(pkg, path_param)
    pkg = rospkg.RosPack().get_path("f_quadruped_control")
    return os.path.join(pkg, "policies", "policy_flat_pushing_pt2.onnx")

class PolicyNodeReal:

    # ---------------------------
    # INITIALIZATION
    # ---------------------------

    def __init__(self):
        rospy.init_node("policy_node_real")

        # ---------------------------
        # PARAMETERS
        # ---------------------------

        self.bypass_base_lin_vel_calc = rospy.get_param("~bypass_base_lin_vel_calc", True)
        self.bypass_base_gravity_trans = rospy.get_param("~bypass_base_gravity_trans", True)

        self.joint_order = rospy.get_param("~joint_order", [
            "FL_HAA", "FL_HFE", "FL_KFE",
            "FR_HAA", "FR_HFE", "FR_KFE",
            "HL_HAA", "HL_HFE", "HL_KFE", "HL_AFE",
            "HR_HAA", "HR_HFE", "HR_KFE", "HR_AFE",
        ])

        self.joint_order_obs = rospy.get_param("~joint_order_obs", [
            "FL_HAA", "FR_HAA", "HL_HAA", "HR_HAA",
            "FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE",
            "FL_KFE", "FR_KFE", "HL_KFE", "HR_KFE",
            "HL_AFE", "HR_AFE",
        ])

        self.model_q0 = rospy.get_param("~model_q0", {
            "FL_HAA": 0.0, "FR_HAA": 0.0, "HL_HAA": 0.0, "HR_HAA": 0.0,
            "FL_HFE": 0.4102, "FR_HFE": 0.4102, "HL_HFE": -0.6981, "HR_HFE": -0.6981,
            "FL_KFE": -1.2716, "FR_KFE": -1.2716, "HL_KFE": 1.676, "HR_KFE": 1.676,
            "HL_AFE": -1.7219, "HR_AFE": -1.7219,
        })

        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.output_topic = rospy.get_param("~output_topic", "/joint_targets_rl")
        self.episode_len_s = float(rospy.get_param("~episode_len_s", 6.0))
        self.action_order_model = rospy.get_param("~action_order_model", list(self.joint_order_obs))

        self.latest_imu = None
        self.latest_cmd = np.zeros(4, dtype=np.float32)
        self.latest_joint_state = None
        self.latest_height_scan_raw = np.zeros(187, dtype=np.float32)
        self.latest_ceiling_height_scan_raw = np.zeros(187, dtype=np.float32)
        self.last_action_model = np.zeros(14, dtype=np.float32)

        self.episode_start = None
        self.policy_started = False

        self.policy_lock = threading.RLock()

        self.policies_config_path = rospy.get_param(
            "~policies_config_path",
            os.path.join(rospkg.RosPack().get_path("f_quadruped_control"), "config", "policies.yaml")
        )

        self.policies = {}
        self.active_policy_id = None
        self.active_policy = None

        self.load_policies_config()
        self.switch_policy(rospy.get_param("~initial_policy_name", "flat"))

        self.imu_acc_buffer = []
        self.imu_time_buffer = []
        self.imu_est_lin_vel = np.zeros(3, dtype=np.float32)

        self.export_obs_csv = rospy.get_param("~export_obs_csv", True)
        self.obs_csv_path = rospy.get_param("~obs_csv_path", os.path.expanduser("~/policy_obs_log.csv"))
        self.obs_csv_file = None
        self.obs_csv_writer = None
        self.obs_csv_header_written = False

        if self.export_obs_csv:
            try:
                csv_dir = os.path.dirname(self.obs_csv_path)
                if csv_dir:
                    os.makedirs(csv_dir, exist_ok=True)
                self.obs_csv_file = open(self.obs_csv_path, "w", newline="")
                self.obs_csv_writer = csv.writer(self.obs_csv_file)
                rospy.on_shutdown(self.close_obs_csv)
            except Exception as exc:
                rospy.logerr("Failed to open OBS CSV file '%s': %s", self.obs_csv_path, str(exc))
                self.export_obs_csv = False

        # ---------------------------
        # ROS Interface
        # ---------------------------
        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=10)

        rospy.Subscriber("/imu", Imu, self.cb_imu, queue_size=50)
        rospy.Subscriber("/cmd", Float32MultiArray, self.cb_cmd, queue_size=10)
        rospy.Subscriber("/joint_states", JointState, self.cb_joint_states, queue_size=50)
        rospy.Subscriber("/lidar", LaserScan, self.cb_lidar, queue_size=10)
        rospy.Subscriber("/switch_cp", String, self.cb_switch_cp, queue_size=10)

        rospy.loginfo("policy_node_real ready: active_policy=%s, hz=%.1f", self.active_policy_id, self.rate_hz)

        self.loop()

    # ---------------------------
    # ROS CALLBACK FUNCTIONS
    # ---------------------------

    def cb_imu(self, msg):
        self.latest_imu = msg

    def cb_cmd(self, msg):
        cmd = np.zeros(4, dtype=np.float32)
        n = min(4, len(msg.data))
        if n > 0:
            cmd[:n] = np.asarray(msg.data[:n], dtype=np.float32)
        self.latest_cmd = cmd

    def cb_joint_states(self, msg):
        self.latest_joint_state = msg

    def cb_lidar(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return
        ranges[~np.isfinite(ranges)] = 0.0

        with self.policy_lock:
            active_policy_id = self.active_policy_id

        if active_policy_id == "crouch":
            if ranges.size < 188:
                rospy.logwarn_throttle(2.0, "Crouch policy expects 188 lidar values")
                return
            self.latest_ceiling_height_scan_raw = ranges[:187].astype(np.float32)
            self.latest_height_scan_raw = ranges[187:188].astype(np.float32)
        else:
            self.latest_height_scan_raw = ranges.astype(np.float32)

    def cb_switch_cp(self, msg):
        policy_name = msg.data.strip()
        self.switch_policy(policy_name)

    # ---------------------------
    # POLICY FUNCTIONS
    # ---------------------------

    def load_policies_config(self):
        with open(self.policies_config_path, "r") as f:
            data = yaml.safe_load(f)

        for policy_name, cfg in data["policies"].items():
            policy_name = str(policy_name)
            cfg["model_path"] = resolve_model_path(cfg["model_path"])
            cfg["model_format"] = cfg.get("model_format", "onnx").strip().lower()
            cfg["ort_input_name"] = cfg.get("ort_input_name", None)
            cfg["action_mode"] = cfg.get("action_mode", "delta").strip().lower()
            cfg["action_scale"] = float(cfg.get("action_scale", 1.0))
            cfg["obs_dim"] = int(cfg["obs_dim"])
            cfg["joint_order_obs"] = cfg.get("joint_order_obs", self.joint_order_obs)
            cfg["action_order_model"] = cfg.get("action_order_model", self.action_order_model)
            cfg["model_q0"] = cfg.get("model_q0", self.model_q0)
            cfg["model"] = None
            cfg["ort_session"] = None

            if cfg["model_format"] == "onnx":
                cfg["ort_session"] = ort.InferenceSession(cfg["model_path"], providers=["CPUExecutionProvider"])
                if cfg["ort_input_name"] is None:
                    cfg["ort_input_name"] = cfg["ort_session"].get_inputs()[0].name
            elif cfg["model_format"] == "torchscript":
                cfg["model"] = torch.jit.load(cfg["model_path"], map_location="cpu")
                cfg["model"].eval()
            else:
                raise RuntimeError("Unknown model_format: %s" % cfg["model_format"])

            self.policies[policy_name] = cfg

    def switch_policy(self, policy_name):
        with self.policy_lock:
            policy_name = str(policy_name).strip()
            if policy_name not in self.policies:
                return
            if policy_name == self.active_policy_id:
                return

            self.active_policy_id = policy_name
            self.active_policy = self.policies[policy_name]
            self.joint_order_obs = self.active_policy["joint_order_obs"]
            self.action_order_model = self.active_policy["action_order_model"]
            self.model_q0 = self.active_policy["model_q0"]
            self.last_action_model = np.zeros(len(self.action_order_model), dtype=np.float32)
            self.episode_start = None
            self.policy_started = False

    def run_policy(self, obs):
        policy = self.active_policy
        if policy["ort_session"] is not None:
            out = policy["ort_session"].run(None, {policy["ort_input_name"]: obs.reshape(1, -1)})[0]
            return np.asarray(out, dtype=np.float32).reshape(-1)
        if policy["model"] is not None:
            x = torch.from_numpy(obs).unsqueeze(0)
            return policy["model"](x).detach().cpu().numpy().astype(np.float32).reshape(-1)
        raise RuntimeError("policy is not loaded")

    # -------------------------------------------
    # POLICY OBSERVATIONS FUNCTIONS AND CSV
    # -------------------------------------------

    def build_obs(self):
        q, dq = self.joint_pos_vel()
        height_scan_dim = int(self.active_policy.get("height_scan_dim", 187))
        ceiling_height_scan_dim = int(self.active_policy.get("ceiling_height_scan_dim", 187))

        height_scan = self.resize_vector(self.latest_height_scan_raw, height_scan_dim)
        ceiling_height_scan = self.resize_vector(self.latest_ceiling_height_scan_raw, ceiling_height_scan_dim)

        term_values = {
            "base_lin_vel": self.imu_base_lin_vel(),
            "base_ang_vel": self.imu_base_ang_vel(),
            "projected_gravity": self.imu_projected_gravity(),
            "pose_commands": self.latest_cmd,
            "joint_pos": q,
            "joint_vel": dq,
            "actions": self.last_action_model,
            "height_scan": height_scan,
            "ceiling_height_scan": ceiling_height_scan,
            "time_remaining_s": np.array([0.0], dtype=np.float32),
        }

        obs_parts = []
        for term in self.active_policy["obs_terms"]:
            obs_parts.append(term_values[term])

        obs = np.concatenate(obs_parts).astype(np.float32)
        return obs

    def obs_ready(self, obs):
        if self.latest_imu is None: return False
        if self.latest_joint_state is None: return False
        if not np.any(np.abs(obs) > 1e-6): return False
        q, dq = self.joint_pos_vel()
        if not np.any(np.abs(q) > 1e-6): return False
        return True

    def inject_time_remaining(self, obs):
        obs = obs.copy()
        time_index = 0
        q_dim = {
            "base_lin_vel": 3, "base_ang_vel": 3, "projected_gravity": 3, "pose_commands": 4,
            "joint_pos": 14, "joint_vel": 14, "actions": len(self.action_order_model),
            "height_scan": int(self.active_policy.get("height_scan_dim", 187)),
            "ceiling_height_scan": int(self.active_policy.get("ceiling_height_scan_dim", 187)),
            "time_remaining_s": 1,
        }
        for term in self.active_policy["obs_terms"]:
            if term == "time_remaining_s":
                obs[time_index] = self.time_remaining()[0]
                return obs
            time_index += q_dim[term]
        return obs

    def close_obs_csv(self):
        if self.obs_csv_file is not None:
            self.obs_csv_file.flush()
            self.obs_csv_file.close()
            self.obs_csv_file = None

    def log_obs_csv(self, obs, q_cmd):
        if not self.export_obs_csv or self.obs_csv_writer is None:
            return
        stamp = rospy.Time.now().to_sec()
        if not self.obs_csv_header_written:
            header = ["timestamp_s"] + ["obs_{}".format(i) for i in range(len(obs))] + ["cmd_" + name for name in self.joint_order]
            self.obs_csv_writer.writerow(header)
            self.obs_csv_header_written = True

        row = [stamp] + obs.astype(float).tolist() + np.asarray(q_cmd, dtype=float).tolist()
        self.obs_csv_writer.writerow(row)

    # -------------------------------------------
    # IMU FUNCTIONS
    # -------------------------------------------

    def imu_base_lin_vel(self):
        if self.latest_imu is None:
            return self.imu_est_lin_vel.copy()

        a_msg = self.latest_imu.linear_acceleration
        if self.bypass_base_lin_vel_calc:
            return np.array([a_msg.x, a_msg.y, a_msg.z], dtype=np.float32)

        now = self.latest_imu.header.stamp
        if now.to_sec() == 0.0: now = rospy.Time.now()
        acc = np.array([a_msg.x, a_msg.y, a_msg.z], dtype=np.float32)

        self.imu_time_buffer.append(now)
        self.imu_acc_buffer.append(acc)

        if len(self.imu_time_buffer) > 3:
            self.imu_time_buffer.pop(0)
            self.imu_acc_buffer.pop(0)

        if len(self.imu_time_buffer) < 3: return self.imu_est_lin_vel.copy()

        t0, t1, t2 = self.imu_time_buffer
        a0, a1, a2 = self.imu_acc_buffer

        dt01, dt12, dt_total = (t1 - t0).to_sec(), (t2 - t1).to_sec(), (t2 - t0).to_sec()

        if dt01 <= 0.0 or dt12 <= 0.0 or dt_total <= 0.0 or dt_total > 0.2:
            self.imu_time_buffer, self.imu_acc_buffer = [now], [acc]
            return self.imu_est_lin_vel.copy()

        ratio = dt01 / dt12
        if ratio < 0.5 or ratio > 2.0:
            self.imu_time_buffer, self.imu_acc_buffer = [self.imu_time_buffer[-1]], [self.imu_acc_buffer[-1]]
            return self.imu_est_lin_vel.copy()

        self.imu_est_lin_vel += (dt_total / 6.0) * (a0 + 4.0 * a1 + a2)
        self.imu_est_lin_vel *= np.exp(-dt_total / 2.0)
        self.imu_time_buffer, self.imu_acc_buffer = [t2], [a2]

        return self.imu_est_lin_vel.copy()

    def imu_base_ang_vel(self):
        if self.latest_imu is None: return np.zeros(3, dtype=np.float32)
        w = self.latest_imu.angular_velocity
        return np.array([w.x, w.y, w.z], dtype=np.float32)

    def imu_projected_gravity(self):
        if self.latest_imu is None: return np.array([0.0, 0.0, -1.0], dtype=np.float32)
        q = self.latest_imu.orientation
        if self.bypass_base_gravity_trans: return np.array([q.x, q.y, q.z], dtype=np.float32)
        R = quat_to_rot(q.w, q.x, q.y, q.z)
        return R.dot(np.array([0.0, 0.0, -1.0], dtype=np.float32)).astype(np.float32)

    # -------------------------------------------
    # OTHER FUNCTIONS
    # -------------------------------------------

    def joint_pos_vel(self):
        q, dq = np.zeros(14, dtype=np.float32), np.zeros(14, dtype=np.float32)
        if self.latest_joint_state is None: return q, dq

        idx = {name: i for i, name in enumerate(self.latest_joint_state.name)}
        for k, name in enumerate(self.joint_order_obs):
            i = idx.get(name)
            if i is None: continue
            if i < len(self.latest_joint_state.position): q[k] = float(self.latest_joint_state.position[i])
            if i < len(self.latest_joint_state.velocity): dq[k] = float(self.latest_joint_state.velocity[i])
        return q, dq

    def time_remaining(self):
        T = max(self.episode_len_s, 1e-3)
        if self.episode_start is None: return np.array([T], dtype=np.float32)
        elapsed = (rospy.Time.now() - self.episode_start).to_sec()
        return np.array([T - (elapsed % T)], dtype=np.float32)

    def resize_vector(self, data, dim):
        data = np.asarray(data, dtype=np.float32).reshape(-1)
        if dim <= 0: return np.zeros(0, dtype=np.float32)
        if data.size == 0: return np.zeros(dim, dtype=np.float32)
        if data.size == dim: return data.astype(np.float32)
        if dim == 1: return np.array([float(np.mean(data))], dtype=np.float32)
        x_old, x_new = np.linspace(0.0, 1.0, data.size), np.linspace(0.0, 1.0, dim)
        return np.interp(x_new, x_old, data).astype(np.float32)

    def base_vector(self):
        return np.array([float(self.model_q0.get(name, 0.0)) for name in self.joint_order], dtype=np.float32)

    def map_model_action_to_control(self, action_model):
        action_model = np.asarray(action_model, dtype=np.float32).reshape(-1)
        by_name = {name: action_model[i] for i, name in enumerate(self.action_order_model)}
        return np.array([float(by_name[name]) for name in self.joint_order], dtype=np.float32)

    def command_to_joint_state(self, positions):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(self.joint_order)
        msg.position = np.asarray(positions, dtype=np.float32).tolist()
        return msg

    # ---------------------------
    # MAIN LOOP
    # ---------------------------

    def loop(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            try:
                with self.policy_lock:
                    obs = self.build_obs()

                    if not self.obs_ready(obs):
                        rate.sleep()
                        continue

                    if not self.policy_started:
                        self.episode_start = rospy.Time.now()
                        self.policy_started = True

                    obs = self.inject_time_remaining(obs)
                    action_model = self.run_policy(obs)
                    self.last_action_model = action_model.copy()

                    action_ctrl = self.map_model_action_to_control(action_model)
                    action_mode = self.active_policy["action_mode"]
                    action_scale = self.active_policy["action_scale"]
                    q0 = self.base_vector()

                if action_mode == "delta": q_cmd = (action_scale * action_ctrl) + q0
                elif action_mode == "absolute": q_cmd = action_ctrl

                self.log_obs_csv(obs,q_cmd)
                self.pub.publish(self.command_to_joint_state(q_cmd))

            except Exception as exc:
                pass
            rate.sleep()

# ---------------------------
# Entrypoint and node startup
# ---------------------------
if __name__ == "__main__":
    try:
        PolicyNodeReal()
    except rospy.ROSInterruptException:
        pass
