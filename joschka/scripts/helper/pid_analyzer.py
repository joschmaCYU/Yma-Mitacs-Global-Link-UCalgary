#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
import threading


class PIDAnalyzer:
    def __init__(self, record_duration=4.0):
        rospy.init_node("pid_analyzer", anonymous=True)
        self.duration = record_duration

        self.lock = threading.Lock()
        self.states = {}  # Stocke les positions réelles et efforts
        self.targets = {}  # Stocke les positions cibles

        self.start_time = None

        # Souscription aux topics
        rospy.Subscriber("/joint_states", JointState, self.state_cb)
        rospy.Subscriber("/joint_targets_rl", JointState, self.target_cb)

        rospy.loginfo(
            f"🟢 Enregistrement des données PID et Effort pendant {self.duration} secondes..."
        )

    def state_cb(self, msg):
        self._process_msg(msg, self.states)

    def target_cb(self, msg):
        self._process_msg(msg, self.targets)

    def _process_msg(self, msg, storage):
        with self.lock:
            if self.start_time is None:
                self.start_time = rospy.Time.now().to_sec()

            t = rospy.Time.now().to_sec() - self.start_time

            for i, name in enumerate(msg.name):
                if not name:
                    continue

                if name not in storage:
                    storage[name] = {"t": [], "pos": [], "eff": []}

                # Récupération de la position
                if i < len(msg.position):
                    storage[name]["t"].append(t)
                    storage[name]["pos"].append(msg.position[i])

                # Récupération de l'effort (couple) s'il est disponible
                if i < len(msg.effort):
                    storage[name]["eff"].append(msg.effort[i])
                else:
                    storage[name]["eff"].append(0.0)

    def analyze_and_plot(self):
        rospy.loginfo(f"⏳ Collecte de données en cours...")
        time.sleep(self.duration)

        rospy.loginfo("📊 Collecte terminée, préparation de l'affichage...")

        # Copie des données pour libérer le thread ROS
        with self.lock:
            targets_copy = {
                k: {"t": list(v["t"]), "pos": list(v["pos"])}
                for k, v in self.targets.items()
            }
            states_copy = {
                k: {"t": list(v["t"]), "pos": list(v["pos"]), "eff": list(v["eff"])}
                for k, v in self.states.items()
            }

        joints = sorted(list(targets_copy.keys()))

        if not joints:
            rospy.logerr("❌ Aucune donnée cible reçue sur /joint_targets_rl !")
            return

        cols = 3
        rows = (len(joints) + cols - 1) // cols
        # On élargit un peu la figure pour que les deux axes Y respirent
        fig, axes = plt.subplots(rows, cols, figsize=(18, 4 * rows))
        fig.subplots_adjust(hspace=0.5, wspace=0.4)

        if isinstance(axes, np.ndarray):
            axes = axes.flatten()
        else:
            axes = [axes]

        for i, jn in enumerate(joints):
            ax = axes[i]

            if (
                jn in targets_copy
                and jn in states_copy
                and len(targets_copy[jn]["t"]) > 0
                and len(states_copy[jn]["t"]) > 0
            ):
                t_targ = np.array(targets_copy[jn]["t"])
                p_targ = np.array(targets_copy[jn]["pos"])

                t_stat = np.array(states_copy[jn]["t"])
                p_stat = np.array(states_copy[jn]["pos"])
                e_stat = np.array(states_copy[jn]["eff"])

                p_stat_interp = np.interp(t_targ, t_stat, p_stat)
                rmse = np.sqrt(np.mean((p_targ - p_stat_interp) ** 2))

                # Tracé des positions (Axe Y de gauche)
                line1 = ax.plot(t_targ, p_targ, "r-", linewidth=1.5, label="Cible (IA)")
                line2 = ax.plot(
                    t_stat, p_stat, "b--", linewidth=1.5, label="Réel (Gazebo)"
                )

                ax.set_title(
                    f"{jn}\nErreur (RMSE): {rmse:.4f}", fontsize=10, fontweight="bold"
                )
                ax.set_xlabel("Temps (s)")
                ax.set_ylabel("Position (rad)")
                ax.grid(True, linestyle=":", alpha=0.7)

                # Tracé de l'effort (Axe Y de droite)
                ax2 = ax.twinx()
                line3 = ax2.plot(
                    t_stat, e_stat, "g:", linewidth=1.5, alpha=0.7, label="Effort (Nm)"
                )
                ax2.set_ylabel("Effort (Nm)", color="g")
                ax2.tick_params(axis="y", labelcolor="g")

                # Regrouper les légendes sur le premier graphique
                if i == 0:
                    lines = line1 + line2 + line3
                    labels = [l.get_label() for l in lines]
                    ax.legend(lines, labels, loc="best")
            else:
                ax.set_title(f"{jn} - Données manquantes")

        # Masquer les graphiques vides
        for j in range(i + 1, len(axes)):
            axes[j].set_visible(False)

        plt.suptitle(
            "Évaluation du suivi de trajectoire PID & Effort Moteur", fontsize=16
        )
        plt.show()


if __name__ == "__main__":
    try:
        analyzer = PIDAnalyzer(record_duration=10.0)
        analyzer.analyze_and_plot()
    except rospy.ROSInterruptException:
        pass
