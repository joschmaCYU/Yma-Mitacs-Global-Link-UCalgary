#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import math
from sensor_msgs.msg import JointState


class QuadrupedStanceTuner:
    def __init__(self, rate_hz=100.0):
        rospy.init_node("stance_tuner", anonymous=True)
        self.pub = rospy.Publisher("/joint_targets_rl", JointState, queue_size=10)
        self.rate_hz = rate_hz

        self.joint_names = [
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

        self.mode = "stop"
        self.start_time = rospy.Time.now().to_sec()
        self.lock = threading.Lock()

        # Thread de publication continue
        self.pub_thread = threading.Thread(target=self.publish_loop)
        self.pub_thread.daemon = True
        self.pub_thread.start()

    def publish_loop(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names

            with self.lock:
                t = rospy.Time.now().to_sec() - self.start_time
                targets = {jn: 0.0 for jn in self.joint_names}

                if self.mode == "squat":
                    # Mouvement de haut en bas sur les 4 pattes pour régler HFE et KFE
                    # Tu peux ajuster les amplitudes ici selon la cinématique de ton robot
                    amp_hfe = 0.4
                    amp_kfe = -0.6
                    amp_afe = 0.3
                    freq = 0.5

                    wave_hfe = amp_hfe * math.sin(2.0 * math.pi * freq * t)
                    wave_kfe = amp_kfe * math.sin(2.0 * math.pi * freq * t)
                    wave_afe = amp_afe * math.sin(2.0 * math.pi * freq * t)

                    for leg in ["FL_", "FR_", "HL_", "HR_"]:
                        targets[leg + "HFE"] = wave_hfe
                        targets[leg + "KFE"] = wave_kfe
                        targets["HR_AFE"] = wave_afe
                        targets["HL_AFE"] = wave_afe

                elif self.mode == "haa_front":
                    for leg in ["FL_", "FR_", "HL_", "HR_"]:
                        targets[leg + "KFE"] = -0.8  # Plie les genoux
                        targets[leg + "HFE"] = 0.5  # Baisse les hanches

                    # Écarter les 3 pattes de soutien vers l'extérieur (0.2 rad) pour l'équilibre
                    targets["FR_HAA"] = -0.4
                    targets["HL_HAA"] = 0.4
                    targets["HR_HAA"] = -0.4

                    targets["FL_HAA"] = 0.4
                    targets["FR_HAA"] = -0.4
                    targets["HR_HAA"] = -0.4

                    # Balayage lent sur la patte Avant-Gauche (FL_HAA)
                    targets["FL_HAA"] = 0.4 * math.sin(2.0 * math.pi * 0.5 * t)

                elif self.mode == "haa_rear":
                    for leg in ["FL_", "FR_", "HL_", "HR_"]:
                        targets[leg + "KFE"] = -0.8
                        targets[leg + "HFE"] = 0.5

                    # Écarter les 3 pattes de soutien vers l'extérieur
                    targets["FL_HAA"] = 0.4
                    targets["FR_HAA"] = -0.4
                    targets["HR_HAA"] = -0.4

                    targets["FR_HAA"] = -0.4
                    targets["HL_HAA"] = 0.4
                    targets["HR_HAA"] = -0.4

                    # Balayage lent sur la patte Arrière-Gauche (HL_HAA)
                    targets["HL_HAA"] = 0.4 * math.sin(2.0 * math.pi * 0.5 * t)

                # Remplir le message dans l'ordre de self.joint_names
                msg.position = [targets[jn] for jn in self.joint_names]

            self.pub.publish(msg)
            rate.sleep()

    def run_cli(self):
        print("\n" + "=" * 50)
        print("🏋️ OUTIL DE TUNING GLOBAL (SQUAT & HAA)")
        print("=" * 50)
        print("Commandes disponibles :")
        print("  squat     : Fait monter et descendre le robot (Règle HFE et KFE)")
        print(
            "  haa_front : Isole l'épaule Avant-Gauche (FL_HAA) avec trépied de soutien"
        )
        print(
            "  haa_rear  : Isole l'épaule Arrière-Gauche (HL_HAA) avec trépied de soutien"
        )
        print("  stop      : Remet le robot en position 0.0 (repos)")
        print("  quit      : Quitter")
        print("=" * 50)

        while not rospy.is_shutdown():
            try:
                cmd = input("\n> ").strip().lower()
                if cmd in ["quit", "exit", "q"]:
                    break
                elif cmd in ["squat", "haa_front", "haa_rear", "stop"]:
                    with self.lock:
                        self.mode = cmd
                        self.start_time = rospy.Time.now().to_sec()
                    print(f"Mode activé : {cmd}")
                else:
                    print("Commande inconnue.")
            except EOFError:
                break


if __name__ == "__main__":
    try:
        tuner = QuadrupedStanceTuner()
        tuner.run_cli()
    except rospy.ROSInterruptException:
        pass
