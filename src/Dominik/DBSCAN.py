import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
import open3d as o3d

file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\pointcloud_tomato.ply"
file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\zugeschnittene_pointcloud.ply"
file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
pcd = o3d.io.read_point_cloud(file_path)

# Extrahieren Sie die Koordinaten der Punkte
points = np.asarray(pcd.points)
sample_rate = 30  # Zum Beispiel jeden 10. Punkt verwenden
reduced_points = points[::sample_rate]

# Verwenden Sie Principal Component Analysis (PCA), um die Hauptkomponenten zu schätzen
pca = PCA(n_components=3)
pca.fit(points)
normal = pca.components_[2, :]

# Schätzen Sie den Radius der Sphären basierend auf der Punktewolke
radius_estimate = 0.3  # Sie müssen möglicherweise den Radius schätzen oder optimieren

# Wenden Sie DBSCAN an, um Sphärenpunkte zu identifizieren
dbscan = DBSCAN(eps=radius_estimate, min_samples=10)
labels = dbscan.fit_predict(points)

# Extrahieren Sie die Punkte, die zur Sphäre gehören
sphere_points = points[labels != -1]

# Visualisieren Sie die Ergebnisse
pcd_sphere = o3d.geometry.PointCloud()
pcd_sphere.points = o3d.utility.Vector3dVector(sphere_points)

o3d.visualization.draw_geometries([pcd_sphere])
