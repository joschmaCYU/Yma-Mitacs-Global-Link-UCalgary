#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, String
import numpy as np

class ElevationMapper:
    def __init__(self):
        rospy.init_node('elevation_mapper_node', anonymous=True)

        self.active_policy = "mixed_terrain"

        # --- CONFIGURATION ---
        self.LIDAR_PITCH_DEG = 90.0 
        
        # --- TEMPORAL SMOOTHING (Low-Pass Filter) ---
        # 1.0 = No smoothing (raw), 0.1 = Very heavy smoothing (slow reaction)
        # 0.2 to 0.4 is usually the sweet spot for walking robots.
        self.alpha = 0.20
        self.previous_grid = None
        self.previous_ceiling = None
        
        # Publishers
        self.pub = rospy.Publisher('/lidar', Float32MultiArray, queue_size=1)
        self.pub_visu = rospy.Publisher('/lidarvisu', PointCloud2, queue_size=1)
        self.pub_cap = rospy.Publisher('/lidarcap', PointCloud2, queue_size=1)
        
        # Subscribers
        self.sub_cloud = rospy.Subscriber('/lidar_relay', PointCloud2, self.cloud_callback, queue_size=1)
        self.sub_policy = rospy.Subscriber('/control_policy', String, self.policy_callback, queue_size=1)

        rospy.loginfo(f"Elevation Mapper initialized (Temporal smoothing active, alpha={self.alpha}).")

    def policy_callback(self, msg):
        self.active_policy = msg.data

    def cloud_callback(self, msg):
        managed_policies = ["mixed_terrain", "mixed terrain, climb, drop, and narrow passage tasks", "crouch"]
        if self.active_policy not in managed_policies:
            return

        # 1. Extract raw points
        cloud = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not cloud: return

        points = np.array(cloud)
        x_raw, y_raw, z_raw = points[:, 0], points[:, 1], points[:, 2]

        # 2. Mathematical Rotation (Pitch)
        theta = np.radians(self.LIDAR_PITCH_DEG)
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        
        x = x_raw * cos_t + z_raw * sin_t
        y = y_raw  
        z = -x_raw * sin_t + z_raw * cos_t

        # --- GEOMETRIC PARAMETERS (3D Bounding Box) ---
        rows, cols = 17, 11
        x_min, x_max = 0.0, 1.6
        y_min, y_max = -0.5, 0.5
        z_min, z_max = -1.5, 0.5  
        lidar_height_offset = 0.5
        resolution = 0.1

        # 3. Strict Filtering (X, Y, AND Z)
        mask_front = (x >= x_min) & (x <= x_max) & (y >= y_min) & (y <= y_max) & (z >= z_min) & (z <= z_max)
        valid_x, valid_y, valid_z = x[mask_front], y[mask_front], z[mask_front]

        # 4. Publish cropped cloud (/lidarcap)
        if len(valid_x) > 0:
            cap_z = valid_z + lidar_height_offset 
            cap_points = np.vstack((valid_x, valid_y, cap_z)).T
            cap_msg = pc2.create_cloud_xyz32(msg.header, cap_points.tolist())
            self.pub_cap.publish(cap_msg)
        else:
            empty_msg = pc2.create_cloud_xyz32(msg.header, [])
            self.pub_cap.publish(empty_msg)

        # 5. Calculate AI Grid
        if len(valid_x) == 0:
            current_grid = np.zeros(rows * cols)
        else:
            heights = valid_z + lidar_height_offset
            heights += np.random.uniform(-0.1, 0.1, size=len(heights))
            heights = np.clip(heights, -1.0, 1.0)

            row_idx = np.round((valid_x - x_min) / resolution).astype(int)
            col_idx = np.round((valid_y - y_min) / resolution).astype(int)
            row_idx = np.clip(row_idx, 0, rows - 1)
            col_idx = np.clip(col_idx, 0, cols - 1)

            flat_idx = row_idx * cols + col_idx
            sums = np.bincount(flat_idx, weights=heights, minlength=rows * cols)
            counts = np.bincount(flat_idx, minlength=rows * cols)

            with np.errstate(divide='ignore', invalid='ignore'):
                current_grid = np.where(counts > 0, sums / counts, 0.0)

        # --- TEMPORAL SMOOTHING APPLICATION (Main Grid) ---
        if self.previous_grid is None:
            self.previous_grid = current_grid
        else:
            current_grid = self.alpha * current_grid + (1.0 - self.alpha) * self.previous_grid
            self.previous_grid = current_grid

        grid_1d = current_grid
        output_data = grid_1d.tolist()

        # 6. SPECIAL CASE: CROUCH
        extra_point_visu = None
        if self.active_policy == "crouch":
            mask_above = (x >= 0.0) & (x <= 0.3) & (y >= -0.3) & (y <= 0.3) & (z > 0.1)
            above_z = z[mask_above]

            if len(above_z) == 0:
                current_ceiling = 1.0 
            else:
                ceiling_raw = above_z + lidar_height_offset
                ceiling_raw += np.random.uniform(-0.1, 0.1, size=len(ceiling_raw))
                current_ceiling = float(np.mean(np.clip(ceiling_raw, -1.0, 1.0)))

            # --- TEMPORAL SMOOTHING APPLICATION (Ceiling) ---
            if self.previous_ceiling is None:
                self.previous_ceiling = current_ceiling
            else:
                self.previous_ceiling = self.alpha * current_ceiling + (1.0 - self.alpha) * self.previous_ceiling
            
            ceiling_height = self.previous_ceiling
            output_data.append(ceiling_height)
            extra_point_visu = ceiling_height
        else:
            # Reset ceiling history if we exit crouch mode
            self.previous_ceiling = None

        # 7. Publish to AI (/lidar)
        grid_msg = Float32MultiArray()
        grid_msg.data = output_data
        self.pub.publish(grid_msg)

        # 8. Publish Visualization (/lidarvisu)
        c, r = np.meshgrid(np.arange(cols), np.arange(rows))
        visu_x = x_min + r.flatten() * resolution
        visu_y = y_min + c.flatten() * resolution
        visu_z = grid_1d

        visu_points = np.vstack((visu_x, visu_y, visu_z)).T

        if extra_point_visu is not None:
            visu_points = np.vstack((visu_points, np.array([0.0, 0.0, extra_point_visu])))

        visu_msg = pc2.create_cloud_xyz32(msg.header, visu_points.tolist())
        self.pub_visu.publish(visu_msg)

if __name__ == '__main__':
    try:
        ElevationMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
