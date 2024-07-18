import open3d as o3d
import numpy as np

def filter_green_points(point_cloud, tolerance):
    # Konvertiere die Pointcloud in ein Numpy-Array
    points = np.asarray(point_cloud.points)
    colors = np.asarray(point_cloud.colors)

    # Extrahiere den Grünkanal der Farben
    green_channel = colors[:, 1]

    # Definiere die Grenzen für den grünen Farbton basierend auf der Toleranz
    lower_green = 0.4 - tolerance
    upper_green = 0.9 + tolerance

    # Finde Indizes der Punkte, die innerhalb der grünen Grenzen liegen
    green_indices = np.where((green_channel >= lower_green) & (green_channel <= upper_green))[0]

    # Filtere die Punkte entsprechend den gefundenen Indizes
    filtered_points = points[green_indices]
    filtered_colors = colors[green_indices]

    # Erstelle eine neue Pointcloud mit den gefilterten Punkten
    filtered_point_cloud = o3d.geometry.PointCloud()
    filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
    filtered_point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)

    file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\dataset\pointcloud\pointcloud_noBackround5_only_close_only_green.ply"
    o3d.io.write_point_cloud(file_path, filtered_point_cloud)

    return filtered_point_cloud


def remove_distant_points(point_cloud, max_distance):
    # Konvertiere die Pointcloud in ein Numpy-Array
    points = np.asarray(point_cloud.points)

    # Berechne die Entfernung jedes Punktes von der Kamera
    distances = np.linalg.norm(points, axis=1)

    # Finde Indizes der Punkte, die innerhalb der maximalen Entfernung liegen
    valid_indices = np.where(distances <= max_distance)[0]

    # Filtere die Punkte entsprechend den gefundenen Indizes
    filtered_points = points[valid_indices]
    filtered_colors = np.asarray(point_cloud.colors)[valid_indices]

    # Erstelle eine neue Pointcloud mit den gefilterten Punkten
    filtered_point_cloud = o3d.geometry.PointCloud()
    filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
    filtered_point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)

    file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\dataset\pointcloud\pointcloud_noBackround9_only_close.ply"
    o3d.io.write_point_cloud(file_path, filtered_point_cloud)

    return filtered_point_cloud


# Dateipfad zur Pointcloud
file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\dataset\pointcloud\pointcloud_noBackround9.ply"

# Toleranz für den grünen Farbton
green_tolerance = 0.0

# Pointcloud einlesen
pcd = o3d.io.read_point_cloud(file_path)
o3d.visualization.draw_geometries([pcd])

filtered_pointcloud = remove_distant_points(pcd, 700)
o3d.visualization.draw_geometries([filtered_pointcloud])

# # Filtere grüne Punkte
# filtered_pcd = filter_green_points(filtered_pointcloud, green_tolerance)

# # Ausgabe der gefilterten Pointcloud
# o3d.visualization.draw_geometries([filtered_pcd])


