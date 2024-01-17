import numpy as np
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
import open3d as o3d

file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\pointcloud_tomato.ply"
file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
# Hier kann eine der oben aufgeführten Dateipfade ausgewählt werden.
pcd = o3d.io.read_point_cloud(file_path)

# Extrahieren Sie die Koordinaten der Punkte
points = np.asarray(pcd.points)
sample_rate = 1  # Zum Beispiel jeden 30. Punkt verwenden
reduced_points = points[::sample_rate]

# Verwenden Sie Principal Component Analysis (PCA), um die Hauptkomponenten zu schätzen
pca = PCA(n_components=3)
pca.fit(points)
normal = pca.components_[2, :]

# Anzahl der Cluster für K-Means
n_clusters = 3  # Sie können die Anzahl der gewünschten Cluster ändern

# Anwenden von K-Means
kmeans = KMeans(n_clusters=n_clusters)
kmeans.fit(points)
labels = kmeans.labels_

# Extrahieren Sie die Punkte für jedes Cluster
cluster_points = [points[labels == i] for i in range(n_clusters)]

# Visualisieren Sie die Ergebnisse
pcd_clusters = [o3d.geometry.PointCloud() for _ in range(n_clusters)]
for i in range(n_clusters):
    pcd_clusters[i].points = o3d.utility.Vector3dVector(cluster_points[i])

o3d.visualization.draw_geometries(pcd_clusters)
