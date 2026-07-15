#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import math
import numpy as np
import traceback

import rospy
import rospkg
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates  
from std_msgs.msg import Float32MultiArray, String

_TORCH_OK = False
_ORT_OK = False
try:
    import torch
    torch.set_grad_enabled(False)
    _TORCH_OK = True
except Exception:
    _TORCH_OK = False

try:
    import onnxruntime as ort
    _ORT_OK = True
except Exception:
    _ORT_OK = False


def quat_to_rot(qw, qx, qy, qz):
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)]
    ], dtype=np.float32)
    return R


class PolicyNodeTorch:
    """
    - Builds observations from /joint_states, /imu, (optional) /odom, (in sim) /gazebo/link_states, and normalized POSE COMMANDS (x, y).
    - Runs the policy (ONNX or TorchScript).
    - Maps action(14) -> control(16).
    - Interprets actions either as DELTAS over q_base (q0 by default) or as ABSOLUTE targets.
    - Publishes /joint_targets_rl.
    - CURRENT: reads profiles from policies.yaml: model_path, ort_input_name, obs_fields, limits,
      action_order_model, action_mode, base_for_delta, model_q0.
    """


    def __init__(self):
        rospy.init_node("policy_node_torch")

        # ---- IMU flags / sign handling ----

        self.flip_gravity_sign = bool(rospy.get_param("~flip_gravity_sign", False))
        self.imu_use_R_transpose = bool(rospy.get_param("~imu_use_R_transpose", True))

        # ---- Remapping / signatures of POSE COMMANDS (x, y) ----

        self.swap_cmd_xy = bool(rospy.get_param("~swap_cmd_xy", False))
        self.invert_cmd_vx = bool(rospy.get_param("~invert_cmd_vx", False))
        self.invert_cmd_vy = bool(rospy.get_param("~invert_cmd_vy", False))

        # ---- Normalization / centering of observations ----

        self.obs_center_q0 = bool(rospy.get_param("~obs_center_q0", True))

        # --------- Basics ----------
        self.model_format = rospy.get_param("~model_format", "onnx").strip().lower()
        self.model_path_par = rospy.get_param("~model_path", "")
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.output_topic = rospy.get_param("~output_topic", "/joint_targets_rl")

        # Sensors / inputs
        self.use_cmd_vel = bool(rospy.get_param("~use_cmd_vel", True)) 
        self.use_odom = bool(rospy.get_param("~use_odom", False))
        self.use_gz_links = bool(rospy.get_param("~use_gazebo_link_states", True))
        self.base_link_name = rospy.get_param("~base_link_name", "base")

        # ----------  Autonomous mode (no joystick) ----------

        self.autonomous_mode = bool(rospy.get_param("~autonomous_mode", False))
        self.auto_cmd_type = rospy.get_param("~auto_cmd_type", "constant").strip().lower()  
        self.auto_x_m = float(rospy.get_param("~auto_x_m", 0.2))
        self.auto_y_m = float(rospy.get_param("~auto_y_m", 0.2))
        self.auto_wz  = float(rospy.get_param("~auto_wz", 0.2))
        self.auto_cycle_s = float(rospy.get_param("~auto_cycle_s", 6.0))
        self._t0 = rospy.Time.now().to_sec()

        # Safety / output filters

        self.clamp_deg = float(rospy.get_param("~clamp_deg", 25.0))
        self.smooth_alpha = float(rospy.get_param("~smooth_alpha", 0.25))
        self.action_gain = float(rospy.get_param("~action_gain", 1.0))

        # Deadman: safety check based on the magnitude of the POSE command

        self.cmd_vel_threshold = float(rospy.get_param("~cmd_vel_threshold", 0.05))
        self.scale_by_cmd = bool(rospy.get_param("~scale_by_cmd", True))
        self.yaw_gain_in_norm = float(rospy.get_param("~yaw_gain_in_norm", 0.5))  

        
        self.hold_mode = rospy.get_param("~hold_mode", "neutral").strip().lower()

        # Neutral posture (fallback)

        self.neutral_positions = rospy.get_param("~neutral_positions", {
            "FL_HAA": 0.0, "FL_HFE": 0.30, "FL_KFE": -0.60,
            "FR_HAA": 0.0, "FR_HFE": 0.30, "FR_KFE": -0.60,
            "HL_HAA": 0.0, "HL_HFE": -0.05, "HL_KFE": 0.45, "HL_AFE": -0.40,
            "HR_HAA": 0.0, "HR_HFE": -0.05, "HR_KFE": 0.45, "HR_AFE": -0.40,
        })


        self.episode_len_s = float(rospy.get_param("~episode_len_s", 6.0))


        self.joint_order = rospy.get_param("~joint_order", [
            "FL_HAA", "FL_HFE", "FL_KFE",
            "FR_HAA", "FR_HFE", "FR_KFE",
            "HL_HAA", "HL_HFE", "HL_KFE", "HL_AFE",
            "HR_HAA", "HR_HFE", "HR_KFE", "HR_AFE",
        ])

        # Observation (14)
        self.joint_order_obs = rospy.get_param("~joint_order_obs", [
            "FL_HAA", "FR_HAA", "HL_HAA", "HR_HAA",
            "FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE",
            "FL_KFE", "FR_KFE", "HL_KFE", "HR_KFE",
            "HL_AFE", "HR_AFE"
        ])


        self.action_order_model = rospy.get_param("~action_order_model", list(self.joint_order_obs))


        self.action_mode = rospy.get_param("~action_mode", "delta").strip().lower()  # delta | absolute
        self.base_for_delta = rospy.get_param("~base_for_delta", "model_q0").strip().lower()

        # Model q0

        self.model_q0 = rospy.get_param("~model_q0", {
            'FL_HAA': 0.0, 'FR_HAA': 0.0, 'HL_HAA': 0.0, 'HR_HAA': 0.0,
            'FL_HFE': 0.4102, 'FR_HFE': 0.4102, 'HL_HFE': -0.6981, 'HR_HFE': -0.6981,
            'FL_KFE': -1.2716, 'FR_KFE': -1.2716, 'HL_KFE': 1.676, 'HR_KFE': 1.676,
            'HL_AFE': -1.7219, 'HR_AFE': -1.7219
        })

        # ---------- POSE COMMANDS in meters ----------

        self.x_max_m = float(rospy.get_param("~x_max_m", 0.5))
        self.y_max_m = float(rospy.get_param("~y_max_m", 0.5))

        # ======== NEW: Profiles from policies.yaml ========

        self.policy_profile = rospy.get_param("~policy_profile", None)
        self.profiles = rospy.get_param("~policy_profiles", {})  
        self.prof = self.profiles.get(self.policy_profile, {}) if self.policy_profile else {}
        self.profile_name = self.policy_profile or "default"


        self.model_format = self.prof.get("model_format", self.model_format)
        self.model_path_par = self.prof.get("model_path", self.model_path_par)
        self._ort_input_name_override = self.prof.get("ort_input_name", None)

        limits = self.prof.get("limits", {})
        if limits:
            self.x_max_m = float(limits.get("x_max_m", self.x_max_m))
            self.y_max_m = float(limits.get("y_max_m", self.y_max_m))

        if "action_order_model" in self.prof:
            self.action_order_model = list(self.prof["action_order_model"])
        if "action_mode" in self.prof:
            self.action_mode = self.prof["action_mode"]
        if "base_for_delta" in self.prof:
            self.base_for_delta = self.prof["base_for_delta"]
        if "model_q0" in self.prof:
            self.model_q0.update(self.prof["model_q0"])

        # Observation scheme (block order)

        self.obs_fields = self.prof.get(
            "obs_fields",
            ["base_lin_vel","base_ang_vel","gravity","pose_xy","zeros2","q14","dq14","last_actions14","time_remaining"]
        )

        # Resolve the model path after applying overrides from configuration

        self.model_path = self._resolve_model_path(self.model_path_par)

        
        self.latest_js = None
        self.latest_imu = None
        self.latest_cmd = Twist()
        self.latest_odom = None
        self.latest_gz = None  # LinkStates
        self.ep_start = rospy.Time.now()

        self.num_ctrl_joints = len(self.joint_order)
        self.last_action_ctrl = np.zeros(self.num_ctrl_joints, dtype=np.float32)
        self.filtered_action = np.zeros(self.num_ctrl_joints, dtype=np.float32)
        self.last_action_model = np.zeros(len(self.action_order_model), dtype=np.float32)

        # ---------- IO ROS ----------
        rospy.Subscriber("/joint_states", JointState, self.cb_joint_states, queue_size=50)
        rospy.Subscriber("/imu", Imu, self.cb_imu, queue_size=50)
        if self.use_odom:
            rospy.Subscriber("/odom", Odometry, self.cb_odom, queue_size=20)
        if self.use_cmd_vel:
            rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd_vel, queue_size=10)
        if self.use_gz_links:
            rospy.Subscriber("/gazebo/link_states", LinkStates, self.cb_gz, queue_size=20)

        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=10)


        self.debug_obs = bool(rospy.get_param("~debug_obs", False))
        if self.debug_obs:
            self.pub_obs_vec = rospy.Publisher("/policy/obs_vec", Float32MultiArray, queue_size=10)
            self.pub_obs_break = rospy.Publisher("/policy/obs_breakdown", String, queue_size=10)


        self.model = None
        self._ort_session = None
        self._ort_input_name = None
        self._load_model()

        rospy.loginfo(
            "policy_node_torch listo. Hz=%.1f, formato=%s, action=%s@%s, pose_norm=[x<=%.2f m, y<=%.2f m], use_odom=%s, use_gz_links=%s, autonomous=%s, profile=%s",
            self.rate_hz, self.model_format, self.action_mode, self.base_for_delta,
            self.x_max_m, self.y_max_m, self.use_odom, self.use_gz_links, str(self.autonomous_mode), str(self.policy_profile)
        )
        self.loop()

    # ========= Callbacks =========
    def cb_joint_states(self, msg):
        self.latest_js = msg

    def cb_imu(self, msg):
        self.latest_imu = msg

    def cb_cmd_vel(self, msg: Twist):
        self.latest_cmd = msg

    def cb_odom(self, msg):
        self.latest_odom = msg

    def cb_gz(self, msg):
        self.latest_gz = msg

    # ========= Util =========
    def _resolve_model_path(self, path_param: str) -> str:
        if not path_param:
            pkg = rospkg.RosPack().get_path("mitacs")
            return os.path.join(pkg, "policies", "policy.onnx")
        if os.path.isabs(path_param):
            return path_param
        pkg = rospkg.RosPack().get_path("mitacs")
        return os.path.join(pkg, path_param)

    def _load_model(self):
        if self.model_format == "onnx":
            if not _ORT_OK:
                rospy.logerr("ONNXRuntime not available. Install: sudo apt-get install python3-onnxruntime")
                return
            try:
                self._ort_session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])

                if self._ort_input_name_override:
                    self._ort_input_name = self._ort_input_name_override
                else:
                    inputs = self._ort_session.get_inputs()
                    if not inputs:
                        raise RuntimeError("ONNX model has no inputs.")
                    self._ort_input_name = inputs[0].name
                rospy.loginfo("✅ ONNX loaded: %s (input='%s')", self.model_path, self._ort_input_name)
            except Exception:
                rospy.logerr("❌ Error loading ONNX:\n%s", traceback.format_exc())
        elif self.model_format == "torchscript":
            if not _TORCH_OK:
                rospy.logerr("PyTorch not available. Install or use formato=onnx.")
                return
            try:
                self.model = torch.jit.load(self.model_path, map_location="cpu")
                self.model.eval()
                rospy.loginfo("✅ TorchScript loaded: %s", self.model_path)
            except Exception:
                rospy.logerr("❌ Error loading TorchScript:\n%s", traceback.format_exc())
        else:
            rospy.logerr("~model_format must be 'onnx' or 'torchscript'.")

    # ======== Señales base ========
    def get_base_lin_vel_body(self):
        """Returns v_base in BODY frame (x, y, z)."""

        if self.use_odom and self.latest_odom is not None:
            v = self.latest_odom.twist.twist.linear
            return np.array([v.x, v.y, v.z], dtype=np.float32)

        if self.use_gz_links and self.latest_gz is not None and self.latest_imu is not None:
            try:
                names = self.latest_gz.name
                idx = -1
                for i, n in enumerate(names):
                    if n.endswith("::" + self.base_link_name) or n == self.base_link_name:
                        idx = i
                        break
                if idx < 0:
                    for i, n in enumerate(names):
                        if n.endswith("::base") or n == "base":
                            idx = i
                            break
                if idx < 0:
                    return np.zeros(3, dtype=np.float32)

                vw = self.latest_gz.twist[idx].linear  
                v_world = np.array([vw.x, vw.y, vw.z], dtype=np.float32)

                q = self.latest_imu.orientation
                R = quat_to_rot(q.w, q.x, q.y, q.z)
                v_body = R.T.dot(v_world) if self.imu_use_R_transpose else R.dot(v_world)
                return v_body.astype(np.float32)
            except Exception:
                return np.zeros(3, dtype=np.float32)

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
        g = R.dot(np.array([0, 0, -1], dtype=np.float32))
        if self.flip_gravity_sign:
            g = -g
        return g.astype(np.float32)

    def _get_pose_cmd_norm(self) -> np.ndarray:
        vx = float(getattr(self.latest_cmd.linear, "x", 0.0)) if self.latest_cmd else 0.0
        vy = float(getattr(self.latest_cmd.linear, "y", 0.0)) if self.latest_cmd else 0.0
        if self.swap_cmd_xy:
            vx, vy = vy, vx
        if self.invert_cmd_vx:
            vx = -vx
        if self.invert_cmd_vy:
            vy = -vy
        arr = np.array([
            vx / max(self.x_max_m, 1e-6),
            vy / max(self.y_max_m, 1e-6)
        ], dtype=np.float32)
        return np.clip(arr, -1.0, 1.0)

    def get_q_dq_obs(self):
        """Returns (q14, dq14) in joint_order_obs; centered if obs_center_q0=True."""

        q_list, dq_list = [], []
        if self.latest_js is None:
            return np.zeros(14, np.float32), np.zeros(14, np.float32)
        idx = {n: i for i, n in enumerate(self.latest_js.name)}
        for jn in self.joint_order_obs:
            if jn in idx:
                i = idx[jn]
                q_val = self.latest_js.position[i] if len(self.latest_js.position) > i else 0.0
                dq_val = self.latest_js.velocity[i] if len(self.latest_js.velocity) > i else 0.0
            else:
                q_val, dq_val = 0.0, 0.0
            if self.obs_center_q0:
                q_val -= float(self.model_q0.get(jn, 0.0))
            q_list.append(q_val); dq_list.append(dq_val)
        return np.array(q_list, np.float32), np.array(dq_list, np.float32)

    def get_feet_contacts_or_zero(self):

        return np.zeros(4, np.float32)

    def get_time_remaining(self):
        elapsed = (rospy.Time.now() - self.ep_start).to_sec()
        T = max(self.episode_len_s, 0.1)
        return max(0.0, T - (elapsed % T))


    def build_obs_from_profile(self):
        """Concatenates blocks defined in self.obs_fields in the order declared by the profile."""


        v_body = self.get_base_lin_vel_body()                 # (3,)
        w_body = self.get_base_ang_vel_body_or_zero()         # (3,)
        grav   = self.get_projected_gravity_or_default()      # (3,)
        pose_xy= self._get_pose_cmd_norm()                    # (2,)
        zeros2 = np.zeros(2, np.float32)                      # (2,)
        q14, dq14 = self.get_q_dq_obs()                       # (14,)(14,)
        last14 = np.array(self.last_action_model, np.float32) # (14,)
        t_rem = np.array([self.get_time_remaining()], np.float32)
        feet4 = self.get_feet_contacts_or_zero()              # (4,)

        bank = {
            "base_lin_vel": v_body,
            "base_ang_vel": w_body,
            "gravity": grav,
            "pose_xy": pose_xy,
            "zeros2": zeros2,
            "q14": q14,
            "dq14": dq14,
            "last_actions14": last14,
            "time_remaining": t_rem,
            "feet_contact4": feet4,
        }

        parts = []
        for key in self.obs_fields:
            if key not in bank:
                rospy.logwarn_throttle(5.0, "obs_fields includes '%s' not implemented; using zero.", key)
                parts.append(np.zeros(1, np.float32))
            else:
                parts.append(bank[key])


        model_dim = None
        try:
            if getattr(self, "_ort_session", None) is not None:
                ishape = self._ort_session.get_inputs()[0].shape  
                if ishape and isinstance(ishape[-1], int):
                    model_dim = int(ishape[-1])
        except Exception:
            model_dim = None

        profile_cfg = self.prof or {}
        profile_dim = profile_cfg.get("obs_pad_to", None)
        auto_pad = bool(profile_cfg.get("obs_auto_pad", True))
        allow_trunc = bool(profile_cfg.get("obs_allow_truncate", False))

        target_dim = profile_dim or model_dim  

        if isinstance(target_dim, int):
            cur = sum(int(x.shape[0]) for x in parts)
            if cur < target_dim:
                if auto_pad:
                    padding = np.zeros((target_dim - cur,), dtype=np.float32)
                    parts.append(padding)
                else:
                    rospy.logerr("Obs size=%d < target=%d y obs_auto_pad=false (perfil=%s)",
                                 cur, target_dim, self.profile_name)
            elif cur > target_dim:
                if allow_trunc:
                    rospy.logwarn("Obs size=%d > target=%d; truncando por obs_allow_truncate=true (perfil=%s)",
                                  cur, target_dim, self.profile_name)
                    flat = np.concatenate(parts, axis=0)[:target_dim]
                    parts = [flat]
                else:
                    rospy.logerr("Obs size=%d > target=%d; ajusta obs_fields o habilita obs_allow_truncate (perfil=%s)",
                                 cur, target_dim, self.profile_name)
        # ---------- FIN PATCH --------------------------------------------

        obs_np = np.concatenate(parts, axis=0).astype(np.float32)


        if self.debug_obs:
            try:
                msg = Float32MultiArray(data=obs_np.tolist())
                self.pub_obs_vec.publish(msg)
                breakdown = {
                    "pose_commands": [float(pose_xy[0]), float(pose_xy[1]), 0.0, 0.0],
                    "obs_fields": list(self.obs_fields),
                    "sizes": [int(x.shape[0]) for x in parts] if isinstance(parts, list) else [int(parts.shape[0])],
                    "q_names": self.joint_order_obs,
                    "dq_names": self.joint_order_obs,
                    "last_actions_names": self.action_order_model,
                    "time_remaining": float(t_rem[0]),
                    "flags": {
                        "use_cmd_vel": self.use_cmd_vel,
                        "use_odom": self.use_odom,
                        "autonomous_mode": self.autonomous_mode,
                    }
                }
                if "base_lin_vel" in self.obs_fields: breakdown["base_lin_vel"] = v_body.tolist()
                if "base_ang_vel" in self.obs_fields: breakdown["base_ang_vel"] = w_body.tolist()
                if "gravity"      in self.obs_fields: breakdown["projected_gravity"] = grav.tolist()
                if "q14"          in self.obs_fields: breakdown["q"]  = q14.tolist()
                if "dq14"         in self.obs_fields: breakdown["dq"] = dq14.tolist()
                if "last_actions14" in self.obs_fields: breakdown["last_actions"] = last14.tolist()

                self.pub_obs_break.publish(String(data=json.dumps(breakdown)))
            except Exception:
                pass

        return obs_np


    def run_policy(self, obs_np):
        if self._ort_session is not None and self._ort_input_name is not None:
            try:
                outputs = self._ort_session.run(None, {self._ort_input_name: obs_np.reshape(1, -1)})
                return np.asarray(outputs[0]).reshape(-1).astype(np.float32)
            except Exception as e:
                rospy.logerr_throttle(2.0, "Error ONNX: %s", repr(e))

        if self.model is not None and _TORCH_OK:
            try:
                x = torch.from_numpy(obs_np).unsqueeze(0)
                y = self.model(x).detach().cpu().numpy().reshape(-1).astype(np.float32)
                return y
            except Exception as e:
                rospy.logerr_throttle(2.0, "Error TorchScript: %s", repr(e))

        # Dummy
        t = rospy.get_time()
        m = len(self.action_order_model)
        return (0.2 * np.sin(2.0 * math.pi * 0.3 * t + np.linspace(0, 2 * math.pi, m))).astype(np.float32)


    def map_model_action_to_control(self, a_model):
        a_model = np.asarray(a_model, dtype=np.float32).reshape(-1)
        M = len(self.action_order_model)
        if a_model.shape[0] != M:
            rospy.logwarn_throttle(2.0, "Model output len=%d, expected=%d. Adjusting.", a_model.shape[0], M)
            if a_model.shape[0] > M:
                a_model = a_model[:M]
            else:
                a_model = np.concatenate([a_model, np.zeros(M - len(a_model), np.float32)])
        ctrl = np.zeros(self.num_ctrl_joints, dtype=np.float32)
        mp = {n: i for i, n in enumerate(self.action_order_model)}
        for ci, cname in enumerate(self.joint_order):
            ctrl[ci] = a_model[mp[cname]] if cname in mp else 0.0
        return ctrl

    def clamp_and_smooth(self, q_target):
        if np.any(~np.isfinite(q_target)):
            rospy.logwarn_throttle(2.0, "Non-finite command. Using last filtered value.")
            return self.filtered_action.copy()
        rad = math.radians(self.clamp_deg)
        q_target = np.clip(q_target, -10 * rad, 10 * rad)
        alpha = float(np.clip(self.smooth_alpha, 0.0, 1.0))
        self.filtered_action = alpha * q_target + (1.0 - alpha) * self.filtered_action
        return self.filtered_action.copy()

    def to_joint_state(self, pos_ctrl):
        out = JointState()
        out.header.stamp = rospy.Time.now()
        out.name = list(self.joint_order)
        out.position = pos_ctrl.tolist()
        return out

    def build_base_vector(self):
        if self.base_for_delta == "model_q0":
            pose = self.model_q0
        elif self.base_for_delta == "neutral":
            pose = self.neutral_positions
        else:
            pose = {}
        q_base = np.zeros(self.num_ctrl_joints, dtype=np.float32)
        for i, jn in enumerate(self.joint_order):
            q_base[i] = float(pose.get(jn, 0.0))
        return q_base

    # ======== Loop ========
    def loop(self):
        rate = rospy.Rate(self.rate_hz)
        q_base = self.build_base_vector()

        while not rospy.is_shutdown():

            if self.autonomous_mode:
                t = rospy.Time.now().to_sec() - self._t0
                vx = self.auto_x_m
                vy = self.auto_y_m
                wz = self.auto_wz
                if self.auto_cmd_type == "sine":
                    phase = 2.0 * math.pi * (t / max(self.auto_cycle_s, 1e-6))
                    vx = self.auto_x_m * math.sin(phase)
                    vy = self.auto_y_m * math.sin(phase)
                    wz = self.auto_wz  * math.sin(phase)
                auto_twist = Twist()
                auto_twist.linear.x = vx
                auto_twist.linear.y = vy
                auto_twist.angular.z = wz
                self.latest_cmd = auto_twist


            obs = self.build_obs_from_profile()
            if obs is None:
                rate.sleep()
                continue


            vx = float(getattr(self.latest_cmd.linear, "x", 0.0)) if self.latest_cmd else 0.0
            vy = float(getattr(self.latest_cmd.linear, "y", 0.0)) if self.latest_cmd else 0.0
            wz = float(getattr(self.latest_cmd.angular, "z", 0.0)) if self.latest_cmd else 0.0
            cmd_mag = math.sqrt(vx * vx + vy * vy + (self.yaw_gain_in_norm * wz) ** 2)

            rospy.loginfo_throttle(1.0, "POSE_CMD(raw): x=%.2f m, y=%.2f m, wz=%.2f", vx, vy, wz)
            try:
                if "pose_xy" in self.obs_fields:
                    pose_xy = self._get_pose_cmd_norm()
                    rospy.loginfo_throttle(1.0, "OBS pose_cmd_norm: [%.2f, %.2f, 0.00, 0.00]", pose_xy[0], pose_xy[1])
            except Exception:
                pass

            if (not self.autonomous_mode) and self.use_cmd_vel and cmd_mag < self.cmd_vel_threshold:
                out = q_base.copy() if self.hold_mode == "neutral" else np.zeros(self.num_ctrl_joints, dtype=np.float32)
                self.last_action_ctrl = out.copy()
                self.filtered_action = out.copy()
                self.pub.publish(self.to_joint_state(out))
                rate.sleep()
                continue



            if self._ort_session is not None:
                try:
                    exp = self._ort_session.get_inputs()[0].shape
                    exp_dim = exp[-1] if exp and isinstance(exp[-1], int) else None
                except Exception:
                    exp_dim = None
                if exp_dim is not None and obs.shape[0] != exp_dim:
                    rospy.logerr_throttle(2.0, "Obs size=%d != esperado por el modelo=%d (perfil=%s). Revisa 'obs_fields'.",
                                          obs.shape[0], exp_dim, self.policy_profile)
                    rate.sleep()
                    continue

            a_model = self.run_policy(obs)


            self.last_action_model = np.asarray(a_model, dtype=np.float32).copy()
            rospy.loginfo_throttle(1.0, "POLICY: actions=%s", np.array2string(self.last_action_model, precision=3))

            a_ctrl = self.map_model_action_to_control(a_model)


            if self.scale_by_cmd:
                try:
                    pose_xy = self._get_pose_cmd_norm()
                    xy_norm = float(math.sqrt(pose_xy[0] ** 2 + pose_xy[1] ** 2))
                except Exception:
                    xy_norm = 0.0
                scale = max(0.0, min(xy_norm, 1.0))
                a_ctrl *= scale


            if self.action_mode == "delta":
                q_cmd = q_base + (self.action_gain * a_ctrl)
            else:
                q_cmd = self.action_gain * a_ctrl

            q_cmd = self.clamp_and_smooth(q_cmd)
            self.last_action_ctrl = q_cmd.copy()
            self.pub.publish(self.to_joint_state(q_cmd))
            rate.sleep()


if __name__ == "__main__":
    try:
        PolicyNodeTorch()
    except rospy.ROSInterruptException:
        pass

