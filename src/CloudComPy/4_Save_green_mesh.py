import os
import open3d as o3d
import numpy as np
from scipy.spatial import KDTree

# Funktion zum Überprüfen des Grünanteils der Mesh-Punkte
def has_sufficient_green(mesh, threshold=0.3):
    # Mesh-Farben extrahieren
    colors = np.asarray(mesh.colors)

    # Zählen der grünen Punkte
    green_count = sum(1 for color in colors if color[1] > color[0] and color[1] > color[2])

    # Berechnen des Anteils grüner Punkte
    green_ratio = green_count / len(colors)

    # Überprüfen, ob der grüne Anteil über dem Schwellenwert liegt
    return green_ratio >= threshold

# Funktion zum Anpassen der Farben in der Mesh-Datei basierend auf der Pointcloud
def adjust_colors(mesh, pointcloud_tree, pointcloud_colors, threshold=0.02):
    # Mesh-Punkte und Farben extrahieren
    mesh_points = np.asarray(mesh.points)
    mesh_colors = np.asarray(mesh.colors)
    
    # Durchlaufen der Punkte im Mesh
    for i, point in enumerate(mesh_points):
        # Überprüfen, ob der Punkt in der Pointcloud vorhanden ist
        distance, index = pointcloud_tree.query(point)
        
        # Überprüfen, ob der Abstand unter dem Schwellenwert liegt
        if distance < threshold:
            # Farbe der großen Pointcloud verwenden, um die Farbe des Punktes in der Mesh-Datei zu ändern
            mesh_colors[i] = pointcloud_colors[index]
    
    # Aktualisieren der Farben in der Mesh-Datei
    mesh.colors = o3d.utility.Vector3dVector(mesh_colors)
    return mesh

# Funktion zur Anpassung der Farben in der Pointcloud und Speicherung der angepassten Meshes
def pointcloud_red_mesh_normal(folder_path, pointcloud_file_path, i):
    # Laden der Pointcloud
    pointcloud = o3d.io.read_point_cloud(pointcloud_file_path)
    pointcloud.paint_uniform_color([0, 1, 0])  # Pointcloud wird grün eingefärbt

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
            mesh_file_path = os.path.join(folder_path, filename)
            
            # Laden der Mesh-Datei als Pointcloud
            mesh = o3d.io.read_point_cloud(mesh_file_path)
            mesh = adjust_colors(mesh, pointcloud_tree1, pointcloud_colors1)
            # Überprüfen, ob die Mesh-Datei genügend grüne Punkte hat
            if has_sufficient_green(mesh):
                # Farben aus der Pointcloud in der Mesh-Datei anpassen
                

                # Definieren des Speicherpfads für die angepasste Mesh-Datei
                output_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color_green\{filename}"
                new_folder_path = os.path.dirname(output_file_path)
                if not os.path.exists(new_folder_path):
                    os.makedirs(new_folder_path)
                
                # Speichern der angepassten Mesh-Datei mit den neuen Farben
                o3d.io.write_point_cloud(output_file_path, mesh)
                print("########################################################")
                print(f"Mesh-Datei gespeichert: {output_file_path}")


# Pfade angeben
for i in range(1, 251):    
    folder_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color"
    pointcloud_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\cropped_pointcloud\pointcloud_copped_{i}.ply"

    # Funktion aufrufen
    pointcloud_red_mesh_normal(folder_path, pointcloud_file_path, i)
