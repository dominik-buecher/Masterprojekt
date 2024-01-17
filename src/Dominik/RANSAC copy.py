import numpy as np
from sklearn.linear_model import RANSACRegressor
from sklearn.decomposition import PCA
import open3d as o3d

file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\pointcloud_tomato.ply"
file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
# Hier kann eine der oben aufgeführten Dateipfade ausgewählt werden.
pcd = o3d.io.read_point_cloud(file_path)

# Extrahieren Sie die Koordinaten der Punkte
points = np.asarray(pcd.points)
sample_rate = 2  # Zum Beispiel jeden 30. Punkt verwenden
reduced_points = points[::sample_rate]

# Verwenden Sie Principal Component Analysis (PCA), um die Hauptkomponenten zu schätzen
pca = PCA(n_components=3)
pca.fit(points)
normal = pca.components_[2, :]

# Verwenden Sie RANSAC, um eine Ebene zu fitten
ransac = RANSACRegressor()
ransac.fit(points[:, :2], points[:, 2])

# Extrahieren Sie die Punkte, die zur Ebene gehören
inlier_mask = ransac.inlier_mask_
plane_points = points[inlier_mask]

# Visualisieren Sie die Ergebnisse
pcd_plane = o3d.geometry.PointCloud()
pcd_plane.points = o3d.utility.Vector3dVector(plane_points)

o3d.visualization.draw_geometries([pcd_plane])
