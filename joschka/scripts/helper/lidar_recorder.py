#!/usr/bin/env python3
import rospy
import csv
import os
from std_msgs.msg import Float32MultiArray

class LidarRecorder:
    def __init__(self):
        rospy.init_node('lidar_recorder', anonymous=True)
        self.latest_data = None
        self.recorded_data = []
        self.max_frames = 50 * 10  # 50 frames à 50Hz = 1 seconde

        rospy.Subscriber('/lidar', Float32MultiArray, self.callback)
        
        rospy.loginfo("En attente des données sur /lidar...")
        while self.latest_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("Début de l'enregistrement (1 seconde à 50Hz)...")
        
        # Timer strict à 50Hz (0.02s)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)

    def callback(self, msg):
        # Met à jour la dernière grille reçue
        self.latest_data = msg.data

    def timer_callback(self, event):
        if len(self.recorded_data) < self.max_frames:
            self.recorded_data.append(self.latest_data)
        else:
            self.timer.shutdown()
            self.save_to_csv()
            rospy.signal_shutdown("Enregistrement terminé.")

    def save_to_csv(self):
        # Sauvegarde dans le dossier courant d'exécution
        filename = os.path.join(os.getcwd(), "real_lidar_50hz.csv")
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            # En-tête : C0, C1, C2... jusqu'à 186 (ou 187 avec le crouch)
            writer.writerow([f"C{i}" for i in range(len(self.recorded_data[0]))])
            writer.writerows(self.recorded_data)
        rospy.loginfo(f"Fichier sauvegardé : {filename}")

if __name__ == '__main__':
    try:
        LidarRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
