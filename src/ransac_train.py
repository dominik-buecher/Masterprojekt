import open3d as o3d
import open3d as o3d
from sklearn.linear_model import RANSACRegressor
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from joblib import dump, load
# pcd = o3d.io.read_point_cloud("data/pointcloud_white_3_46.ply")
# plane_model, inliers = pcd.segment_plane(distance_threshold=100, ransac_n=4, num_iterations=1000)


# inlier_cloud = pcd.select_by_index(inliers)
# outlier_cloud = pcd.select_by_index(inliers, invert=True)

# inlier_cloud.paint_uniform_color([1, 0, 0])
# outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])

# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

import os

def extract_grapes(folder_path):
    # Liste für alle geladenen Pointclouds erstellen
    all_grape_points = []
    all_grape_colors = []

    # Durch alle Dateien im Ordner iterieren
    for filename in os.listdir(folder_path):
        if filename.endswith(".ply"):  # Nur .ply-Dateien berücksichtigen
            pointcloud_file = os.path.join(folder_path, filename)

            pointcloud = o3d.io.read_point_cloud(pointcloud_file)
            points = np.asarray(pointcloud.points)
            colors = np.asarray(pointcloud.colors)

            # Invertiere die y-Koordinaten
            points[:, 1:] *= -1
            var = (min(points[:, 2]) * -1) - 1000
            # Bereich einschränken
            front_points = points[points[:, 2] > min(points[:, 2]) + var]
            front_colors = colors[points[:, 2] > min(points[:, 2]) + var]

            # Füge die aktuellen Punkte zur Gesamtliste hinzu
            all_grape_points.append(front_points)
            all_grape_colors.append(front_colors)

    # Konvertiere die Gesamtliste zu numpy-Arrays
    all_grape_points = np.concatenate(all_grape_points, axis=0)
    all_grape_colors = np.concatenate(all_grape_colors, axis=0)

    # RANSAC
    ransac = RANSACRegressor()
    ransac.fit(all_grape_points[:, :2], all_grape_points[:, 2])
    epsilon = 1000  # => Schwelle für die Einbeziehung von Punkten als Inliers

    dump(ransac, 'model/ransac_model.joblib')
    # Filtere Punkte basierend auf RANSAC
    inlier_mask = (
        (front_points[:, 2] > ransac.predict(front_points[:, :2]) - epsilon) &
        (front_points[:, 2] < ransac.predict(front_points[:, :2]) + epsilon)
    )

    grape_points = front_points[inlier_mask]
    grape_colors = front_colors[inlier_mask]

    grape_pointcloud = o3d.geometry.PointCloud()
    grape_pointcloud.points = o3d.utility.Vector3dVector(grape_points)
    grape_pointcloud.colors = o3d.utility.Vector3dVector(grape_colors)

    o3d.visualization.draw_geometries([grape_pointcloud])
    o3d.io.write_point_cloud("data/trauben_pointcloud.ply", grape_pointcloud)


if __name__ == "__main__":
    pointcloud_file = "data/temp"
    extract_grapes(pointcloud_file)