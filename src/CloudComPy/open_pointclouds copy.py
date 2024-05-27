import os
import open3d as o3d
import numpy as np
from scipy.spatial import KDTree

# Pfad zu dem Ordner, in dem sich die Pointcloud und die Mesh-Dateien befinden
folder_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data"
# Pfad zur Pointcloud-Datei
pointcloud_file_path = os.path.join(folder_path, "pointcloud.ply")

# Funktion, um Farben der Pointcloud auf Basis der Mesh-Farben anzupassen
def adjust_pointcloud_colors(pointcloud, mesh, pointcloud_tree, color_distance_threshold=0.01):
    # Erstellen eines KD-Baums für die Pointcloud
    pointcloud_colors = np.asarray(pointcloud.colors)
    
    # Mesh-Punkte und -Farben extrahieren
    mesh_points = np.asarray(mesh.points)
    mesh_colors = np.asarray(mesh.colors)

    # Durchlaufen der Punkte im Mesh
    for i, point in enumerate(mesh_points):
        # Überprüfen, ob der Punkt in der Pointcloud vorhanden ist
        distance, index = pointcloud_tree.query(point)
        
        # Überprüfen, ob der Abstand unter dem Schwellenwert liegt
        if distance < color_distance_threshold:
            # Farbe des Punktes in der Pointcloud auf die tatsächliche Farbe des Punktes aus der Mesh-Datei setzen
            pointcloud_colors[index] = mesh_colors[i]
    
    # Aktualisierung der Farben in der Pointcloud
    pointcloud.colors = o3d.utility.Vector3dVector(pointcloud_colors)

# Funktion, um die Pointcloud und die Mesh-Dateien zu verarbeiten
def process_pointcloud_and_mesh(folder_path, pointcloud_file_path, pointcloud_color, mesh_adjust_color=True):
    # Laden der Pointcloud
    pointcloud = o3d.io.read_point_cloud(pointcloud_file_path)
    pointcloud.paint_uniform_color(pointcloud_color)  # Pointcloud einfärben

    # Erstellen eines KD-Baums für die Pointcloud
    pointcloud_points = np.asarray(pointcloud.points)
    pointcloud_tree = KDTree(pointcloud_points)

    # Durchlaufen der Dateien im Ordner und Überprüfen der Mesh-Dateien
    for filename in os.listdir(folder_path):
        if filename.startswith("mesh_cloud") and filename.endswith(".ply"):
            mesh_file_path = os.path.join(folder_path, filename)
            
            # Laden der Mesh-Datei als Pointcloud
            mesh = o3d.io.read_point_cloud(mesh_file_path)
            
            # Mesh-Farben anpassen
            if mesh_adjust_color:
                adjust_pointcloud_colors(pointcloud, mesh, pointcloud_tree)

    # Visualisierung der Pointcloud
    o3d.visualization.draw_geometries([pointcloud])

# Aufrufen der Funktionen mit den jeweiligen Einstellungen
process_pointcloud_and_mesh(folder_path, pointcloud_file_path, [0, 1, 0])  # Pointcloud grün einfärben
process_pointcloud_and_mesh(folder_path, pointcloud_file_path, [1, 0, 0], mesh_adjust_color=True)  # Pointcloud rot einfärben und Farben aus Mesh verwenden
process_pointcloud_and_mesh(folder_path, pointcloud_file_path, [1, 0, 0], mesh_adjust_color=False)  # Pointcloud rot einfärben und Farben aus Mesh verwenden
process_pointcloud_and_mesh(folder_path, pointcloud_file_path, [0.5, 0.5, 0.5])  # Pointcloud in Grau einfärben
