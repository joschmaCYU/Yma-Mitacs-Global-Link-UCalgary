#!/usr/bin/env python3
import rospy
import numpy as np
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray


class GridExtractor:
    def __init__(self):
        rospy.init_node("rl_grid_extractor", anonymous=True)

        # Dimensions requises par l'environnement RL
        self.target_rows = 17  # X (Avant/Arrière)
        self.target_cols = 11  # Y (Gauche/Droite)

        self.pub = rospy.Publisher("/lidar", Float32MultiArray, queue_size=1)
        self.sub = rospy.Subscriber(
            "/elevation_mapping/elevation_map", GridMap, self.map_callback
        )
        rospy.loginfo("Extracteur RL prêt (17x11). En attente des données...")

    def map_callback(self, msg):
        try:
            # 1. Identifier l'index de la couche d'élévation
            layer_idx = msg.layers.index("elevation")

            # 2. Lire les dimensions dynamiques
            size_x = msg.data[layer_idx].layout.dim[0].size
            size_y = msg.data[layer_idx].layout.dim[1].size

            # 3. Conversion en array Numpy (GridMap est stocké en Column-Major 'F')
            raw_data = np.array(msg.data[layer_idx].data)

            # 4. Nettoyage pour le RL : remplacer les NaN par le niveau du sol (0.0)
            raw_data = np.nan_to_num(raw_data, nan=0.0)
            grid = raw_data.reshape((size_x, size_y), order="F")

            # 5. Dérouler le buffer circulaire de GridMap
            # inner_start_index = décalage sur X (lignes), outer_start_index = décalage sur Y (colonnes)
            grid = np.roll(grid, -msg.inner_start_index, axis=0)
            grid = np.roll(grid, -msg.outer_start_index, axis=1)

            # 6. Extraire le centre exact (position du robot)
            start_x = (size_x - self.target_rows) // 2
            start_y = (size_y - self.target_cols) // 2

            sub_grid = grid[
                start_x : start_x + self.target_rows,
                start_y : start_y + self.target_cols,
            ]

            # 7. Aplatir (Row-Major 'C' par défaut) et publier
            out_msg = Float32MultiArray()
            out_msg.data = sub_grid.flatten(order="C").tolist()
            self.pub.publish(out_msg)

        except ValueError:
            rospy.logwarn_throttle(
                5.0, "La couche 'elevation' n'est pas encore initialisée."
            )
        except Exception as e:
            rospy.logerr_throttle(2.0, f"Erreur lors de l'extraction : {e}")


if __name__ == "__main__":
    try:
        GridExtractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
