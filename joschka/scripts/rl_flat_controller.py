#!/usr/bin/env python3
import rospy
import numpy as np
import onnxruntime as ort
from tf.transformations import quaternion_matrix
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class FlatRLController:
    def __init__(self):
        rospy.init_node("rl_onnx_controller", anonymous=True)

        # --- CHARGEMENT DU MODÈLE ONNX (POLITIQUE FLAT) ---
        # TODO: Remplacer le chemin par celui de ta politique "flat"
        onnx_path = rospy.get_param(
            "~onnx_model_path",
            "/root/catkin_ws/src/mitacs/Policy/Exported_policies/policy.onnx",
        )
        self.ort_session = ort.InferenceSession(onnx_path)
        self.input_name = self.ort_session.get_inputs()[0].name

        # --- PARAMÈTRES ET BUFFERS ---
        self.rl_joint_names = [
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
        ]

        # Scales d'entraînement
        self.lin_vel_scale = 2.0
        self.ang_vel_scale = 0.25
        self.dof_pos_scale = 1.0
        self.dof_vel_scale = 0.05
        self.action_scale = 0.5

        # Pose de repos (IsaacLabRef)
        self.default_dof_pos = np.array(
            [
                0.0,
                0.0,
                0.0,
                0.0,  # HAA
                0.4102,
                0.4102,
                -0.6981,
                -0.6981,  # HFE
                -1.2716,
                -1.2716,
                1.676,
                1.676,  # KFE
                -1.7219,
                -1.7219,  # AFE
            ]
        )

        # Buffers pour les 56 dimensions
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)
        self.projected_gravity = np.array([0.0, 0.0, -1.0])
        # Commande: avancer à 0.5 m/s
        self.pose_commands = np.array([0.5, 0.0, 0.0, 0.5])
        self.joint_pos = np.zeros(14)
        self.joint_vel = np.zeros(14)
        self.last_actions = np.zeros(14)
        self.time_remaining_s = np.array([1.0])

        # --- SUBSCRIBERS ---
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        # Note: Le subscriber Lidar a été supprimé

        # --- PUBLISHERS ---
        self.joint_pubs = []
        for rl_name in self.rl_joint_names:
            topic_name = f"/{rl_name}_position_controller/command"
            self.joint_pubs.append(rospy.Publisher(topic_name, Float64, queue_size=1))

        rospy.loginfo("En attente des capteurs (Odom, Joints)...")
        rospy.sleep(1.0)

        rospy.loginfo("Démarrage du cerveau ONNX (FLAT - 56 dims) à 50Hz !")
        self.timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)

    def odom_callback(self, msg):
        self.base_lin_vel[0] = msg.twist.twist.linear.x * self.lin_vel_scale
        self.base_lin_vel[1] = msg.twist.twist.linear.y * self.lin_vel_scale
        self.base_lin_vel[2] = msg.twist.twist.linear.z * self.lin_vel_scale

        self.base_ang_vel[0] = msg.twist.twist.angular.x * self.ang_vel_scale
        self.base_ang_vel[1] = msg.twist.twist.angular.y * self.ang_vel_scale
        self.base_ang_vel[2] = msg.twist.twist.angular.z * self.ang_vel_scale

        q = msg.pose.pose.orientation
        rot_matrix = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        gravity_world = np.array([0.0, 0.0, -1.0])
        self.projected_gravity = np.dot(rot_matrix.T, gravity_world)

    def joint_callback(self, msg):
        for i, rl_name in enumerate(self.rl_joint_names):
            if rl_name in msg.name:
                idx = msg.name.index(rl_name)
                self.joint_pos[i] = (
                    msg.position[idx] - self.default_dof_pos[i]
                ) * self.dof_pos_scale
                self.joint_vel[i] = msg.velocity[idx] * self.dof_vel_scale

    def control_loop(self, event):
        # 1. Assemblage du vecteur d'observation (Taille = 56)
        obs = np.concatenate(
            [
                self.base_lin_vel,  # 3
                self.base_ang_vel,  # 3
                self.projected_gravity,  # 3
                self.pose_commands,  # 4
                self.joint_pos,  # 14
                self.joint_vel,  # 14
                self.last_actions,  # 14
                self.time_remaining_s,  # 1
            ]
        ).astype(np.float32)

        # Vérification de sécurité (optionnelle, pour le debug)
        if obs.shape[0] != 56:
            rospy.logerr_throttle(
                1.0, f"Erreur de dimension ! Attendu 56, reçu {obs.shape[0]}"
            )
            return

        obs_batch = np.expand_dims(obs, axis=0)

        # 2. Inférence ONNX
        actions = self.ort_session.run(None, {self.input_name: obs_batch})[0]
        self.last_actions = actions.flatten()

        # 3. Application de l'action
        for i, action in enumerate(self.last_actions):
            target_position = self.default_dof_pos[i] + (action * self.action_scale)
            msg = Float64()
            msg.data = target_position
            self.joint_pubs[i].publish(msg)


if __name__ == "__main__":
    try:
        FlatRLController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
