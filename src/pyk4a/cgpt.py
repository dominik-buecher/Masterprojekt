import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
import open3d as o3d
import pyk4a
from pyk4a import Config, PyK4A



k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_720P,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=True,
        )
    )

k4a.start()

while True:
    capture = k4a.get_capture()



    transformation = pyk4a.Transformation(Config)

    # Konvertieren Sie Tiefen- und Farbdaten in NumPy-Arrays
    depth_image = capture.depth.data
    color_image = capture.color.data

    # Konvertieren Sie die Tiefendaten in Punkte
    point_cloud = transformation.depth_image_to_point_cloud(depth_image)

    # Visualisieren Sie die Punktewolke
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :, :3])
    o3d.visualization.draw_geometries([pcd])

    # Extrahieren Sie die Koordinaten der Punkte
    points = point_cloud[:, :, :3].reshape(-1, 3)

    # Verwenden Sie Principal Component Analysis (PCA), um die Hauptkomponenten zu schätzen
    pca = PCA(n_components=3)
    pca.fit(points)
    normal = pca.components_[2, :]

    # Schätzen Sie den Radius der Sphären basierend auf der Punktewolke
    radius_estimate = 0.1  # Sie müssen möglicherweise den Radius schätzen oder optimieren

    # Wenden Sie DBSCAN an, um Sphärenpunkte zu identifizieren
    dbscan = DBSCAN(eps=radius_estimate, min_samples=10)
    labels = dbscan.fit_predict(points)

    # Extrahieren Sie die Punkte, die zur Sphäre gehören
    sphere_points = points[labels != -1]

    # Visualisieren Sie die Ergebnisse
    pcd_sphere = o3d.geometry.PointCloud()
    pcd_sphere.points = o3d.utility.Vector3dVector(sphere_points)

    o3d.visualization.draw_geometries([pcd_sphere])
