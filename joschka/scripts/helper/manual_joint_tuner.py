#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import math
from sensor_msgs.msg import JointState


class SingleJointTuner:
    def __init__(self, rate_hz=100.0):
        rospy.init_node("single_joint_tuner", anonymous=True)
        self.pub = rospy.Publisher("/joint_targets_rl", JointState, queue_size=10)
        self.rate_hz = rate_hz

        # Liste standard de tes 12 moteurs
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

        # État par défaut : tout à zéro
        self.targets = {name: 0.0 for name in self.joint_names}

        # Paramètres pour le générateur de mouvement (sinusoïde)
        self.active_sine_joint = None
        self.sine_amp = 0.0
        self.sine_freq = 0.0
        self.start_time = rospy.Time.now().to_sec()

        self.lock = threading.Lock()

        # Le thread de publication tourne en tâche de fond à 100 Hz
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
                # Calcul du temps écoulé pour la sinusoïde
                t = rospy.Time.now().to_sec() - self.start_time
                current_targets = []

                for jn in self.joint_names:
                    if jn == self.active_sine_joint:
                        # Génère l'onde sur l'articulation active
                        val = self.sine_amp * math.sin(
                            2.0 * math.pi * self.sine_freq * t
                        )
                        current_targets.append(val)
                    else:
                        current_targets.append(self.targets[jn])

                msg.position = current_targets

            self.pub.publish(msg)
            rate.sleep()

    def print_menu(self):
        print("\n" + "=" * 40)
        print("🤖 OUTIL DE CALIBRATION PID MANUEL")
        print("=" * 40)
        print("Commandes :")
        print(
            "  set <articulation> <valeur>   : Fixe une cible statique (ex: set FL_HFE 0.5)"
        )
        print(
            "  sine <articulation> <amp> <f> : Mouvement sinusoïdal (ex: sine FL_KFE 0.5 1.0)"
        )
        print("  zero                          : Remet tous les moteurs à 0.0")
        print("  stop                          : Arrête l'onde sinusoïdale")
        print("  quit                          : Quitter le programme")
        print("=" * 40)

    def run_cli(self):
        self.print_menu()
        while not rospy.is_shutdown():
            try:
                cmd_line = input("\n> ").strip().split()
                if not cmd_line:
                    continue

                cmd = cmd_line[0].lower()

                if cmd in ["quit", "exit", "q"]:
                    break

                elif cmd == "zero":
                    with self.lock:
                        self.targets = {name: 0.0 for name in self.joint_names}
                        self.active_sine_joint = None
                    print("Toutes les articulations remises à 0.0.")

                elif cmd == "stop":
                    with self.lock:
                        if self.active_sine_joint:
                            self.targets[self.active_sine_joint] = 0.0
                            self.active_sine_joint = None
                    print("Mouvement arrêté et retour à 0.0.")

                elif cmd == "set":
                    if len(cmd_line) != 3:
                        print("Erreur: format -> set <articulation> <valeur>")
                        continue
                    jn, val = cmd_line[1], cmd_line[2]
                    if jn not in self.joint_names:
                        print(f"Erreur: '{jn}' n'est pas reconnu.")
                        continue
                    try:
                        val_f = float(val)
                        with self.lock:
                            self.targets[jn] = val_f
                            if self.active_sine_joint == jn:
                                self.active_sine_joint = None
                        print(f"[{jn}] -> {val_f} rad")
                    except ValueError:
                        print("Erreur: la valeur doit être un nombre.")

                elif cmd == "sine":
                    if len(cmd_line) != 4:
                        print(
                            "Erreur: format -> sine <articulation> <amplitude> <frequence>"
                        )
                        continue
                    jn, amp, freq = cmd_line[1], cmd_line[2], cmd_line[3]
                    if jn not in self.joint_names:
                        print(f"Erreur: '{jn}' n'est pas reconnu.")
                        continue
                    try:
                        amp_f, freq_f = float(amp), float(freq)
                        with self.lock:
                            self.active_sine_joint = jn
                            self.sine_amp = amp_f
                            self.sine_freq = freq_f
                        print(f"[{jn}] -> Sinusoïde Amp={amp_f} Freq={freq_f}Hz")
                    except ValueError:
                        print(
                            "Erreur: amplitude et fréquence doivent être des nombres."
                        )

                else:
                    print("Commande invalide.")

            except EOFError:
                break
            except Exception as e:
                print(f"Erreur inattendue: {e}")


if __name__ == "__main__":
    try:
        tuner = SingleJointTuner()
        tuner.run_cli()
    except rospy.ROSInterruptException:
        pass
