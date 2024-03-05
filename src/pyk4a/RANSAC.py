import open3d as o3d
from sklearn.linear_model import RANSACRegressor
import numpy as np
from joblib import dump, load

def extract_grapes(pointcloud_file):
    pointcloud = o3d.io.read_point_cloud(pointcloud_file)
    plane_model, inliers = pointcloud.segment_plane(distance_threshold=250, ransac_n=4, num_iterations=10000)
    inlier_cloud = pointcloud.select_by_index(inliers)

    points = np.asarray(inlier_cloud.points)
    colors = np.asarray(inlier_cloud.colors)

    # Invertiere die y-Koordinaten
    points[:, 1:] *= -1

    # Bereich einschränken
    var = (min(points[:, 2]) * -1) - 1000
    front_points = points[points[:, 2] > min(points[:, 2]) + var]
    front_colors = colors[points[:, 2] > min(points[:, 2]) + var]

    # Farbfilterung - Beispiel: Grüne Trauben
    green_threshold = 0.7
    is_green = front_colors[:, 1] > green_threshold

    # Kombinieren Sie Farb- und RANSAC-Filter
    filtered_points = front_points[is_green]
    filtered_colors = front_colors[is_green]

    # RANSAC
    ransac = RANSACRegressor(min_samples=10, residual_threshold=500, max_trials=5000)
    ransac.fit(filtered_points[:, :2], filtered_points[:, 2])

    # Filtere Punkte basierend auf RANSAC
    inlier_mask = (
        (filtered_points[:, 2] > ransac.predict(filtered_points[:, :2]) - 500) &
        (filtered_points[:, 2] < ransac.predict(filtered_points[:, :2]) + 500)
    )

    grape_points = filtered_points[inlier_mask]
    grape_colors = filtered_colors[inlier_mask]

    grape_pointcloud = o3d.geometry.PointCloud()
    grape_pointcloud.points = o3d.utility.Vector3dVector(grape_points)
    grape_pointcloud.colors = o3d.utility.Vector3dVector(grape_colors)

    o3d.visualization.draw_geometries([grape_pointcloud])
    o3d.io.write_point_cloud("data/trauben_pointcloud.ply", grape_pointcloud)

if __name__ == "__main__":
    pointcloud_file = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
    extract_grapes(pointcloud_file)






# import open3d as o3d
# from sklearn.linear_model import RANSACRegressor
# import numpy as np
# from joblib import dump, load

# def extract_grapes(pointcloud_file):
#     pointcloud = o3d.io.read_point_cloud(pointcloud_file)
#     plane_model, inliers = pointcloud.segment_plane(distance_threshold=250, ransac_n=4, num_iterations=10000)
#     inlier_cloud = pointcloud.select_by_index(inliers)

#     points = np.asarray(inlier_cloud.points)
#     colors = np.asarray(inlier_cloud.colors)

#     # Invertiere die y-Koordinaten
#     points[:, 1:] *= -1

#     # Bereich einschränken
#     var = (min(points[:, 2]) * -1) - 1000
#     front_points = points[points[:, 2] > min(points[:, 2]) + var]
#     front_colors = colors[points[:, 2] > min(points[:, 2]) + var]

#     # RANSAC
#     ransac = RANSACRegressor(min_samples=10, residual_threshold=500, max_trials=5000)
#     ransac.fit(front_points[:, :2], front_points[:, 2])

#     # Filtere Punkte basierend auf RANSAC
#     inlier_mask = (
#         (front_points[:, 2] > ransac.predict(front_points[:, :2]) - 500) &
#         (front_points[:, 2] < ransac.predict(front_points[:, :2]) + 500)
#     )

#     grape_points = front_points[inlier_mask]
#     grape_colors = front_colors[inlier_mask]

#     grape_pointcloud = o3d.geometry.PointCloud()
#     grape_pointcloud.points = o3d.utility.Vector3dVector(grape_points)
#     grape_pointcloud.colors = o3d.utility.Vector3dVector(grape_colors)

#     o3d.visualization.draw_geometries([grape_pointcloud])
#     o3d.io.write_point_cloud("data/trauben_pointcloud.ply", grape_pointcloud)

# if __name__ == "__main__":
#     pointcloud_file = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
#     extract_grapes(pointcloud_file)





# import open3d as o3d
# import numpy as np
# from sklearn.linear_model import RANSACRegressor
# from sklearn.preprocessing import StandardScaler

# def ransac_cone(pcd, apex_threshold=0.005, angle_threshold=np.radians(10), min_samples=10, max_trials=1000):
#     """
#     RANSAC für kegelförmige Objekte.
    
#     Parameters:
#     - pcd: Punktwolke (PointCloud)
#     - apex_threshold: Schwellenwert für die Residuen des Kegelspitze (float)
#     - angle_threshold: Schwellenwert für den Öffnungswinkel des Kegels (in Radian, float)
#     - min_samples: Mindestanzahl von Punkten für die Schätzung eines Kegels (int)
#     - max_trials: Maximale Anzahl von RANSAC-Versuchen (int)
    
#     Returns:
#     - inlier_mask: Boolesche Maske für Inliers (Liste von bools)
#     """
#     # Konvertieren Sie die Punktwolke in ein NumPy-Array
#     points = np.asarray(pcd.points)

#     # Standardisieren Sie die Punkte für eine bessere Schätzung
#     scaler = StandardScaler()
#     scaled_points = scaler.fit_transform(points)

#     # Verwenden Sie RANSAC, um einen Kegel zu fitten
#     ransac = RANSACRegressor(residual_threshold=apex_threshold, min_samples=min_samples, max_trials=max_trials)
#     ransac.fit(scaled_points[:, :3], np.linalg.norm(scaled_points[:, :3], axis=1))

#     # Extrahieren Sie die Inliers
#     inlier_mask = ransac.inlier_mask_

#     # Überprüfen Sie den Öffnungswinkel des geschätzten Kegels
#     cone_axis = ransac.estimator_.coef_[:3]
#     cone_angle = np.arccos(np.abs(np.dot(cone_axis, [0, 0, 1])))
    
#     # Filtern Sie die Inliers basierend auf dem Öffnungswinkel
#     inlier_mask = np.logical_and(inlier_mask, cone_angle < angle_threshold)

#     return inlier_mask

# # Beispielverwendung:
# file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
# pcd = o3d.io.read_point_cloud(file_path)

# # Wenden Sie RANSAC an
# inlier_mask = ransac_cone(pcd)

# # Extrahieren Sie die Punkte, die zum geschätzten Kegel gehören
# inlier_points = np.asarray(pcd.points)[inlier_mask]

# # Visualisieren Sie die Ergebnisse
# pcd_inliers = o3d.geometry.PointCloud()
# pcd_inliers.points = o3d.utility.Vector3dVector(inlier_points)

# o3d.visualization.draw_geometries([pcd_inliers])













# import open3d as o3d
# import numpy as np
# from sklearn.linear_model import RANSACRegressor
# from sklearn.preprocessing import StandardScaler

# def ransac_sphere(pcd, radius_threshold=0.005, min_samples=20, max_trials=5000):
#     """
#     RANSAC für spezielle Sphären.
    
#     Parameters:
#     - pcd: Punktwolke (PointCloud)
#     - radius_threshold: Radius-Schwellenwert für Inliers (float)
#     - min_samples: Mindestanzahl von Punkten für die Schätzung einer Sphäre (int)
#     - max_trials: Maximale Anzahl von RANSAC-Versuchen (int)
    
#     Returns:
#     - inlier_mask: Boolesche Maske für Inliers (Liste von bools)
#     """
#     # Konvertieren Sie die Punktwolke in ein NumPy-Array
#     points = np.asarray(pcd.points)

#     # Standardisieren Sie die Punkte für eine bessere Schätzung
#     scaler = StandardScaler()
#     scaled_points = scaler.fit_transform(points)

#     # Verwenden Sie RANSAC, um eine Kugel (Sphäre) zu fitten
#     ransac = RANSACRegressor(residual_threshold=radius_threshold, min_samples=min_samples, max_trials=max_trials)
#     ransac.fit(scaled_points[:, :3], np.linalg.norm(scaled_points[:, :3], axis=1))

#     # Extrahieren Sie die Inliers
#     inlier_mask = ransac.inlier_mask_

#     return inlier_mask

# # Beispielverwendung:
# file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
# pcd = o3d.io.read_point_cloud(file_path)

# # Wenden Sie RANSAC an
# inlier_mask = ransac_sphere(pcd)

# # Extrahieren Sie die Punkte, die zur geschätzten Sphäre gehören
# inlier_points = np.asarray(pcd.points)[inlier_mask]

# # Visualisieren Sie die Ergebnisse
# pcd_inliers = o3d.geometry.PointCloud()
# pcd_inliers.points = o3d.utility.Vector3dVector(inlier_points)

# o3d.visualization.draw_geometries([pcd_inliers])












# import numpy as np
# from sklearn.linear_model import RANSACRegressor
# import open3d as o3d

# file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
# pcd = o3d.io.read_point_cloud(file_path)

# # Extrahieren Sie die Koordinaten der Punkte
# points = np.asarray(pcd.points)
# sample_rate = 1
# reduced_points = points[::sample_rate]

# # Der Durchmesser der Sphäre (2 * Radius) beträgt 1 cm
# target_diameter = 0.01

# # Verwenden Sie RANSAC, um eine Kugel (Sphäre) zu fitten
# ransac = RANSACRegressor(residual_threshold=target_diameter / 2.0)
# ransac.fit(points[:, :3], np.linalg.norm(points[:, :3], axis=1))

# # Extrahieren Sie die Punkte, die zur Kugel gehören (Inliers)
# inlier_mask = ransac.inlier_mask_
# sphere_points = points[inlier_mask]

# # Visualisieren Sie die Ergebnisse
# pcd_sphere = o3d.geometry.PointCloud()
# pcd_sphere.points = o3d.utility.Vector3dVector(sphere_points)

# o3d.visualization.draw_geometries([pcd_sphere])













# import numpy as np
# from sklearn.linear_model import RANSACRegressor
# from sklearn.decomposition import PCA
# import open3d as o3d

# file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\pointcloud_tomato.ply"
# file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
# # Hier kann eine der oben aufgeführten Dateipfade ausgewählt werden.
# pcd = o3d.io.read_point_cloud(file_path)

# # Extrahieren Sie die Koordinaten der Punkte
# points = np.asarray(pcd.points)
# sample_rate = 2  # Zum Beispiel jeden 30. Punkt verwenden
# reduced_points = points[::sample_rate]

# # Verwenden Sie Principal Component Analysis (PCA), um die Hauptkomponenten zu schätzen
# pca = PCA(n_components=3)
# pca.fit(points)
# normal = pca.components_[2, :]

# # Verwenden Sie RANSAC, um eine Ebene zu fitten
# ransac = RANSACRegressor()
# ransac.fit(points[:, :2], points[:, 2])

# # Extrahieren Sie die Punkte, die zur Ebene gehören
# inlier_mask = ransac.inlier_mask_
# plane_points = points[inlier_mask]

# # Visualisieren Sie die Ergebnisse
# pcd_plane = o3d.geometry.PointCloud()
# pcd_plane.points = o3d.utility.Vector3dVector(plane_points)

# o3d.visualization.draw_geometries([pcd_plane])
