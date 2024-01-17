import numpy as np
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
import open3d as o3d

file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
pcd = o3d.io.read_point_cloud(file_path)

# Extrahieren Sie die Koordinaten der Punkte
points = np.asarray(pcd.points)
sample_rate = 1
reduced_points = points[::sample_rate]

# Verwenden Sie Principal Component Analysis (PCA), um die Hauptkomponenten zu schätzen
pca = PCA(n_components=3)
pca.fit(points)
normal = pca.components_[2, :]

# Anzahl der Cluster für K-Means
n_clusters = 3

# Anwenden von K-Means
kmeans = KMeans(n_clusters=n_clusters)
kmeans.fit(points)
labels = kmeans.labels_

# Extrahieren Sie die Punkte für jedes Cluster
cluster_points = [points[labels == i] for i in range(n_clusters)]

# Filtern Sie runde Objekte basierend auf der Geometrie
filtered_clusters = []
for i in range(n_clusters):
    cluster_pcd = o3d.geometry.PointCloud()
    cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points[i])

    # Berechnen Sie die Ausdehnung (Extent) der Punktewolke
    extent = cluster_pcd.get_axis_aligned_bounding_box().get_extent()

    # Hier können Sie eine Schwellenwert-Bedingung festlegen, um runde Objekte zu filtern
    is_round = extent[0] / extent[1] > 1.5 # Beispiel-Schwellenwert

    if not is_round:
        filtered_clusters.append(cluster_pcd)

# Visualisieren Sie die Ergebnisse
o3d.visualization.draw_geometries(filtered_clusters)










# import open3d as o3d
# import numpy as np
# from sklearn.linear_model import RANSACRegressor
# from sklearn.preprocessing import StandardScaler

# def ransac_cone(pcd, apex_threshold=0.01, angle_threshold=np.radians(50), min_samples=10, max_trials=1000):
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

