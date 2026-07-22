#!/usr/bin/env python3
import rospy
import numpy as np
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


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
        self.pub_vis_lidar = rospy.Publisher("/lidar_visu", PointCloud2, queue_size=1)
        rospy.loginfo("Extracteur RL prêt (17x11). En attente des données...")

    def map_callback(self, msg):
        try:
            # 1. Identifier l'index de la couche d'élévation
            layer_idx = msg.layers.index("elevation")

            # 2. Lire les dimensions dynamiques et la résolution physique
            size_x = msg.data[layer_idx].layout.dim[0].size
            size_y = msg.data[layer_idx].layout.dim[1].size
            resolution = msg.info.resolution  # Nécessaire pour placer les points en 3D

            # 3. Conversion en array Numpy (GridMap est stocké en Column-Major 'F')
            raw_data = np.array(msg.data[layer_idx].data)

            # 4. Nettoyage pour le RL : remplacer les NaN par le niveau du sol (0.0)
            raw_data = np.nan_to_num(raw_data, nan=0.0)
            grid = raw_data.reshape((size_x, size_y), order="F")

            # 5. Dérouler le buffer circulaire de GridMap
            grid = np.roll(grid, -msg.inner_start_index, axis=0)
            grid = np.roll(grid, -msg.outer_start_index, axis=1)

            # 6. Extraire le centre exact (position du robot)
            start_x = (size_x - self.target_rows) // 2
            start_y = (size_y - self.target_cols) // 2

            sub_grid = grid[
                start_x : start_x + self.target_rows,
                start_y : start_y + self.target_cols,
            ]

            # Aligner les orientations des axes avec Isaac Lab:
            sub_grid = np.flip(sub_grid, axis=(0, 1))

            # Transposer en (11, 17)
            sub_grid_t = sub_grid.T

            # 7. Aplatir les hauteurs (Z)
            flat_z = sub_grid_t.flatten(order="C")

            # Publier pour la politique
            out_msg = Float32MultiArray()
            out_msg.data = flat_z.tolist()
            self.pub.publish(out_msg)

            # ==========================================
            # 8. CONSTRUCTION DU NUAGE DE POINTS (RVIZ)
            # ==========================================

            # Création des vecteurs X (avant/arrière) et Y (gauche/droite) centrés sur le robot
            # Ils correspondent à la grille dejà "flippée"
            x_coords = (
                np.arange(self.target_rows) - (self.target_rows - 1) / 2.0
            ) * resolution
            y_coords = (
                np.arange(self.target_cols) - (self.target_cols - 1) / 2.0
            ) * resolution

            # On crée une grille 2D (17x11)
            grid_x, grid_y = np.meshgrid(x_coords, y_coords, indexing="ij")

            # On applique EXACTEMENT la même transposition que pour les hauteurs Z
            grid_x_t = grid_x.T
            grid_y_t = grid_y.T

            # On aplatit X et Y de la même façon que Z
            flat_x = grid_x_t.flatten(order="C")
            flat_y = grid_y_t.flatten(order="C")

            # On regroupe [X, Y, Z] pour chaque point
            points_3d = np.column_stack((flat_x, flat_y, flat_z)).tolist()

            # Publication du PointCloud2
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = msg.info.header.frame_id

            if len(points_3d) > 0 and hasattr(self, "pub_vis_lidar"):
                cloud_msg = pc2.create_cloud_xyz32(header, points_3d)
                self.pub_vis_lidar.publish(cloud_msg)

        except Exception as e:
            rospy.logerr(f"Erreur dans map_callback: {e}")


if __name__ == "__main__":
    try:
        GridExtractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
