import os
import open3d as o3d
import numpy as np
from scipy.spatial import KDTree
import re  # Modul für reguläre Ausdrücke

def adjust_colors(mesh, pointcloud_tree, pointcloud_colors, threshold=0.01):
    # Mesh-Punkte und Farben extrahieren
    mesh_points = np.asarray(mesh.points)
    mesh_colors = np.asarray(mesh.colors)
    
    # Durchlaufen der Punkte im Mesh
    for i, point in enumerate(mesh_points):
        # Überprüfen, ob der Punkt in der Pointcloud vorhanden ist
        distance, index = pointcloud_tree.query(point)
        
        # Überprüfen, ob der Abstand unter dem Schwellenwert liegt
        if distance < threshold:
            # Farbe der Pointcloud verwenden, um die Farbe des Punktes in der Mesh-Datei zu ändern
            mesh_colors[i] = pointcloud_colors[index]
    
    # Aktualisieren der Farben in der Mesh-Datei
    mesh.colors = o3d.utility.Vector3dVector(mesh_colors)
    return mesh

def pointcloud_red_mesh_normal(folder_path, pointcloud_file_path, i):
    # Laden der Pointcloud
    pointcloud = o3d.io.read_point_cloud(pointcloud_file_path)
    pointcloud.paint_uniform_color([1, 0, 0])  # Pointcloud wird rot eingefärbt

    # Erstellen eines KD-Baums für die Pointcloud
    pointcloud_points = np.asarray(pointcloud.points)
    pointcloud_tree = KDTree(pointcloud_points)
    pointcloud_colors = np.asarray(pointcloud.colors)

    pointcloud1 = o3d.io.read_point_cloud(pointcloud_file_path)
    # Erstellen eines KD-Baums für die Pointcloud
    pointcloud_points1 = np.asarray(pointcloud1.points)
    pointcloud_tree1 = KDTree(pointcloud_points1)
    pointcloud_colors1 = np.asarray(pointcloud1.colors)

    # Durchlaufen der Dateien im Ordner und Überprüfen der Mesh-Dateien
    for filename in os.listdir(folder_path):
        if filename.startswith("mesh_cloud") and filename.endswith(".ply"):
            # Extrahieren der Zahl aus dem Dateinamen
            match = re.search(r'\d+', filename)
            if match:
                m = match.group()  # Die gefundene Zahl im Dateinamen
            
            mesh_file_path = os.path.join(folder_path, filename)
            
            # Laden der Mesh-Datei als Pointcloud
            mesh = o3d.io.read_point_cloud(mesh_file_path)
            mesh = adjust_colors(mesh, pointcloud_tree1, pointcloud_colors1)
            
            # Erstellen des neuen Pfads mit der extrahierten Zahl m
            new_path = (rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color\{filename}")
            new_folder_path = os.path.dirname(new_path)
            if not os.path.exists(new_folder_path):
                os.makedirs(new_folder_path)
            # Speichern der bearbeiteten Mesh-Datei
            o3d.io.write_point_cloud(new_path, mesh)

            # Weiterer Code zum Verarbeiten der Pointcloud oder Mesh-Datei kann hier eingefügt werden

for i in range(1, 101):
    folder_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh_cloud"
    pointcloud_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\cropped_pointcloud\pointcloud_copped_{i}.ply"

    pointcloud_red_mesh_normal(folder_path, pointcloud_file_path, i)
    i += 1
