#!/usr/bin/env python3
import rospy
import numpy as np
import onnxruntime as ort
from tf.transformations import quaternion_matrix
from std_msgs.msg import Float32MultiArray, Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class RLController:
    def __init__(self):
        rospy.init_node("rl_onnx_controller", anonymous=True)

        # --- CHARGEMENT DU MODÈLE ONNX ---
        # Remplace par le chemin absolu vers ton fichier s'il n'est pas dans le dossier courant
        onnx_path = rospy.get_param(
            "~onnx_model_path",
            "/root/catkin_ws/src/mitacs/Policy/Exported_policies/rough_fixed-slope-2.onnx",
        )
        self.ort_session = ort.InferenceSession(onnx_path)

        # Récupération automatique du nom de la couche d'entrée (souvent "obs" ou "observations")
        self.input_name = self.ort_session.get_inputs()[0].name

        # --- PARAMÈTRES ET BUFFERS ---
        # L'ordre EXACT des 14 joints tel qu'utilisé pendant l'entraînement RL
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



        # Scales (Facteurs de normalisation issus de la configuration d'entraînement)
        self.lin_vel_scale = 2.0
        self.ang_vel_scale = 0.25
        self.dof_pos_scale = 1.0
        self.dof_vel_scale = 0.05
        self.action_scale = 0.5

        # Angles par défaut du robot au repos (default_joint_pos dans Isaac Lab)
        # ordonnés selon self.rl_joint_names
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

        # Buffers pour stocker les dernières valeurs reçues
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)
        self.projected_gravity = np.array([0.0, 0.0, -1.0])  # Gravité par défaut
        self.pose_commands = np.array(
            [0.5, 0.0, 0.0, 0.5]
        )  # [vel_x, vel_y, yaw_vel, height_target] par ex.
        self.joint_pos = np.zeros(14)
        self.joint_vel = np.zeros(14)
        self.last_actions = np.zeros(14)
        self.height_scan = np.zeros(187)
        self.time_remaining_s = np.array([1.0])  # Factice ou géré si épisodique

        # --- SUBSCRIBERS ---
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        rospy.Subscriber("/lidar", Float32MultiArray, self.lidar_callback)

        # --- PUBLISHERS (Pour envoyer les commandes aux moteurs Gazebo) ---
        self.joint_pubs = []
        for rl_name in self.rl_joint_names:
            # Assure-toi que le nom du topic correspond à tes ros_controllers dans Gazebo
            topic_name = f"/{rl_name}_position_controller/command"
            self.joint_pubs.append(rospy.Publisher(topic_name, Float64, queue_size=1))

        # Attente des premières données
        rospy.loginfo("En attente des capteurs (Lidar, Odom, Joints)...")
        rospy.sleep(1.0)

        # --- BOUCLE DE CONTRÔLE (50 Hz = 0.02s) ---
        rospy.loginfo("Démarrage du cerveau ONNX à 50Hz !")
        self.timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)

    def odom_callback(self, msg):
        # 1. Vitesses (linéaire et angulaire)
        self.base_lin_vel[0] = msg.twist.twist.linear.x * self.lin_vel_scale
        self.base_lin_vel[1] = msg.twist.twist.linear.y * self.lin_vel_scale
        self.base_lin_vel[2] = msg.twist.twist.linear.z * self.lin_vel_scale

        self.base_ang_vel[0] = msg.twist.twist.angular.x * self.ang_vel_scale
        self.base_ang_vel[1] = msg.twist.twist.angular.y * self.ang_vel_scale
        self.base_ang_vel[2] = msg.twist.twist.angular.z * self.ang_vel_scale

        # 2. Projection de la gravité
        # L'IMU/Odom donne l'orientation du robot par rapport au monde.
        # Pour projeter la gravité (monde -> base), on inverse la rotation du robot.
        q = msg.pose.pose.orientation
        rot_matrix = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        # Gravité dans le monde = [0, 0, -1]
        gravity_world = np.array([0.0, 0.0, -1.0])
        # R^T * g_w donne la gravité vue par le robot
        self.projected_gravity = np.dot(rot_matrix.T, gravity_world)

    def joint_callback(self, msg):
        # Les joint_states de ROS sont mis dans l'ordre du RL et alignés sur les noms réels
        for i, rl_name in enumerate(self.rl_joint_names):
            if rl_name in msg.name:
                idx = msg.name.index(rl_name)
                # Position (normalisée par rapport à la pose par défaut)
                self.joint_pos[i] = (
                    msg.position[idx] - self.default_dof_pos[i]
                ) * self.dof_pos_scale
                # Vitesse
                self.joint_vel[i] = msg.velocity[idx] * self.dof_vel_scale

    def lidar_callback(self, msg):
        if len(msg.data) == 187:
            self.height_scan = np.array(msg.data)

    def control_loop(self, event):
        # 1. Assemblage du vecteur d'observation (Taille = 243)
        obs = np.concatenate(
            [
                self.base_lin_vel,  # 3
                self.base_ang_vel,  # 3
                self.projected_gravity,  # 3
                self.pose_commands,  # 4
                self.joint_pos,  # 14
                self.joint_vel,  # 14
                self.last_actions,  # 14
                self.height_scan,  # 187
                self.time_remaining_s,  # 1
            ]
        ).astype(np.float32)  # ONNX requiert du float32

        # Redimensionnement en batch_size = 1 -> shape (1, 243)
        obs_batch = np.expand_dims(obs, axis=0)

        # 2. Inférence dans le réseau de neurones
        actions = self.ort_session.run(None, {self.input_name: obs_batch})[0]

        # ONNX renvoie (1, 14), on l'aplatit en (14,)
        self.last_actions = actions.flatten()

        # 3. Application de l'action aux moteurs Gazebo
        for i, action in enumerate(self.last_actions):
            # Le calcul classique : position_cible = position_defaut + (action * echelle_action)
            target_position = self.default_dof_pos[i] + (action * self.action_scale)

            # Publication sur le topic du contrôleur
            msg = Float64()
            msg.data = target_position
            self.joint_pubs[i].publish(msg)


if __name__ == "__main__":
    try:
        RLController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
