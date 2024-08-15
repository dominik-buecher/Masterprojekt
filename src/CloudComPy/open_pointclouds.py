import os
import open3d as o3d
import numpy as np
from scipy.spatial import KDTree

# Funktion zum Überprüfen der grünen Farbanteile im Mesh
def has_sufficient_green(mesh, threshold=0.0, green_tolerance=0.0):
    # Farben der Punkte im Mesh erhalten
    colors = np.asarray(mesh.colors)
    colors_int = (colors * 255).astype(np.uint8)
    
    # Überprüfen, ob Farben im Mesh vorhanden sind
    if len(colors) == 0:
        return False  # Mesh hat keine Farben

    # Zählen der grünen Punkte
    green_count = 0
    for color in colors_int:
        r, g, b = color
        # Überprüfen, ob der Grünanteil über dem Rot- und Blauanteil liegt
        if g > r and g > b:
            green_count += 1

    # Verhältnis der grünen Punkte zur Gesamtanzahl der Punkte berechnen
    total_points = len(colors)
    green_ratio = green_count / total_points

    # Überprüfen, ob der grüne Anteil über dem Schwellenwert liegt
    return green_ratio > threshold


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
            # Farbe der großen Pointcloud verwenden, um die Farbe des Punktes in der Mesh-Datei zu ändern
            mesh_colors[i] = pointcloud_colors[index]
    
    # Aktualisieren der Farben in der Mesh-Datei
    mesh.colors = o3d.utility.Vector3dVector(mesh_colors)
    return mesh




def pointcloud_normal_mesh_red(folder_path, pointcloud_file_path):
    # Laden der Pointcloud
    pointcloud = o3d.io.read_point_cloud(pointcloud_file_path)

    # Erstellen eines KD-Baums für die Pointcloud
    pointcloud_points = np.asarray(pointcloud.points)
    pointcloud_tree = KDTree(pointcloud_points)
    pointcloud_colors = np.asarray(pointcloud.colors)


    # Durchlaufen der Dateien im Ordner und Überprüfen der Mesh-Dateien
    for filename in os.listdir(folder_path):
        if filename.startswith("mesh_cloud") and filename.endswith(".ply"):
            mesh_file_path = os.path.join(folder_path, filename)
            
            # Laden der Mesh-Datei als Pointcloud
            mesh = o3d.io.read_point_cloud(mesh_file_path)

            # Farben in der Mesh-Datei
            mesh_points = np.asarray(mesh.points)
            mesh_colors = np.asarray(mesh.colors)
            
            # Durchlaufen der Punkte in der Mesh-Datei
            for i, point in enumerate(mesh_points):
                # Überprüfen, ob der Punkt in der Pointcloud vorhanden ist
                distance, index = pointcloud_tree.query(point)
                
                # Hier kannst du einen Schwellenwert für die Entfernung festlegen, um zu überprüfen, ob die Punkte identisch sind.
                # Zum Beispiel:
                if distance < 0.01:  # Schwellenwert für den Abgleich
                    # Setzen der Farbe des Punktes in der Pointcloud auf die tatsächliche Farbe aus der Mesh-Datei
                    pointcloud_colors[index] = [1, 0, 0]

    # Aktualisierung der Farben in der Pointcloud
    pointcloud.colors = o3d.utility.Vector3dVector(pointcloud_colors)

    # Visualisierung der Pointcloud
    o3d.visualization.draw_geometries([pointcloud])
    output_file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\pointcloud.ply"
    #o3d.io.write_point_cloud(output_file_path, pointcloud)

def pointcloud_green_mesh_red(folder_path, pointcloud_file_path):
    # Laden der Pointcloud
    pointcloud = o3d.io.read_point_cloud(pointcloud_file_path)
    pointcloud.paint_uniform_color([0, 1, 0])  # Pointcloud wird grün eingefärbt

    # Erstellen eines KD-Baums für die Pointcloud
    pointcloud_points = np.asarray(pointcloud.points)
    pointcloud_tree = KDTree(pointcloud_points)
    pointcloud_colors = np.asarray(pointcloud.colors)


    # Durchlaufen der Dateien im Ordner und Überprüfen der Mesh-Dateien
    for filename in os.listdir(folder_path):
        if filename.startswith("mesh_cloud") and filename.endswith(".ply"):
            mesh_file_path = os.path.join(folder_path, filename)
            
            # Laden der Mesh-Datei als Pointcloud
            mesh = o3d.io.read_point_cloud(mesh_file_path)

            # Farben in der Mesh-Datei
            mesh_points = np.asarray(mesh.points)
            mesh_colors = np.asarray(mesh.colors)
            
            # Durchlaufen der Punkte in der Mesh-Datei
            for i, point in enumerate(mesh_points):
                # Überprüfen, ob der Punkt in der Pointcloud vorhanden ist
                distance, index = pointcloud_tree.query(point)
                
                # Hier kannst du einen Schwellenwert für die Entfernung festlegen, um zu überprüfen, ob die Punkte identisch sind.
                # Zum Beispiel:
                if distance < 0.01:  # Schwellenwert für den Abgleich
                    # Setzen der Farbe des Punktes in der Pointcloud auf die tatsächliche Farbe aus der Mesh-Datei
                    pointcloud_colors[index] = [1, 0, 0]

    # Aktualisierung der Farben in der Pointcloud
    pointcloud.colors = o3d.utility.Vector3dVector(pointcloud_colors)

    # Visualisierung der Pointcloud
    o3d.visualization.draw_geometries([pointcloud])

    

def pointcloud_gray_mesh_depthcolor(folder_path, pointcloud_file_path):
    # Laden der Pointcloud
    pointcloud = o3d.io.read_point_cloud(pointcloud_file_path)
    dunkleres_grau_rgb = [128, 128, 128]  # Mittlerer Grauton

    # Die Farbe der Pointcloud auf dunkleres Grau setzen
    pointcloud.paint_uniform_color(np.array(dunkleres_grau_rgb) / 255.0)
    # Erstellen eines KD-Baums für die Pointcloud
    pointcloud_points = np.asarray(pointcloud.points)
    pointcloud_tree = KDTree(pointcloud_points)
    pointcloud_colors = np.asarray(pointcloud.colors)

    # Durchlaufen der Dateien im Ordner und Überprüfen der Mesh-Dateien
    for filename in os.listdir(folder_path):
        if filename.startswith("mesh_") and filename.endswith(".ply"):
            mesh_file_path = os.path.join(folder_path, filename)
            
            # Laden der Mesh-Datei als Pointcloud
            mesh = o3d.io.read_point_cloud(mesh_file_path)            
            # Farben in der Mesh-Datei
            mesh_points = np.asarray(mesh.points)
            mesh_colors = np.asarray(mesh.colors)
            
            # Durchlaufen der Punkte in der Mesh-Datei
            for i, point in enumerate(mesh_points):
                # Überprüfen, ob der Punkt in der Pointcloud vorhanden ist
                distance, index = pointcloud_tree.query(point)
                
                # Hier kannst du einen Schwellenwert für die Entfernung festlegen, um zu überprüfen, ob die Punkte identisch sind.
                # Zum Beispiel:
                if distance < 0.01:  # Schwellenwert für den Abgleich
                    # Setzen der Farbe des Punktes in der Pointcloud auf die tatsächliche Farbe aus der Mesh-Datei
                    pointcloud_colors[index] = mesh_colors[i]

    # Aktualisierung der Farben in der Pointcloud
    pointcloud.colors = o3d.utility.Vector3dVector(pointcloud_colors)

    # Visualisierung der Pointcloud
    o3d.visualization.draw_geometries([pointcloud])


def pointcloud_red_mesh_normal(folder_path, pointcloud_file_path):
    # Laden der Pointcloud
    pointcloud = o3d.io.read_point_cloud(pointcloud_file_path)
    pointcloud.paint_uniform_color([1, 0, 0])  # Pointcloud wird grün eingefärbt

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
            
            # Farben in der Mesh-Datei
            mesh_points = np.asarray(mesh.points)
            mesh_colors = np.asarray(mesh.colors)

            # Durchlaufen der Punkte in der Mesh-Datei
            for i, point in enumerate(mesh_points):
                # Überprüfen, ob der Punkt in der Pointcloud vorhanden ist
                distance, index = pointcloud_tree.query(point)
                
                # Hier kannst du einen Schwellenwert für die Entfernung festlegen, um zu überprüfen, ob die Punkte identisch sind.
                # Zum Beispiel:
                if distance < 0.01:  # Schwellenwert für den Abgleich
                    # Setzen der Farbe des Punktes in der Pointcloud auf die tatsächliche Farbe aus der Mesh-Datei
                    pointcloud_colors[index] = mesh_colors[i]

    # Aktualisierung der Farben in der Pointcloud
    pointcloud.colors = o3d.utility.Vector3dVector(pointcloud_colors)

    # Visualisierung der Pointcloud
    o3d.visualization.draw_geometries([pointcloud])




def mesh_only_green(folder_path, pointcloud_file_path):
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
            
            # Farben in der Mesh-Datei
            mesh_points = np.asarray(mesh.points)
            mesh_colors = np.asarray(mesh.colors)
            #if True:
            if has_sufficient_green(mesh, threshold=0.2, green_tolerance=0.0):
                # Durchlaufen der Punkte in der Mesh-Datei
                for i, point in enumerate(mesh_points):
                    # Überprüfen, ob der Punkt in der Pointcloud vorhanden ist
                    distance, index = pointcloud_tree.query(point)
                    
                    # Hier kannst du einen Schwellenwert für die Entfernung festlegen, um zu überprüfen, ob die Punkte identisch sind.
                    # Zum Beispiel:
                    if distance < 0.01:  # Schwellenwert für den Abgleich
                        # Setzen der Farbe des Punktes in der Pointcloud auf die tatsächliche Farbe aus der Mesh-Datei
                        pointcloud_colors[index] = [1, 0, 0]

    # Aktualisierung der Farben in der Pointcloud
    pointcloud.colors = o3d.utility.Vector3dVector(pointcloud_colors)
    o3d.visualization.draw_geometries([pointcloud])



# Pfad zu dem Ordner, in dem sich die Pointcloud und die Mesh-Dateien befinden
#folder_path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_10.ply\mesh_cloud_color_green"
folder_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_232.ply\mesh_cloud_color_green"
# folder_path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\CloudComPy\dataset\new\mesh_cloud"

# Pfad zur Pointcloud-Datei
pointcloud_file_path = os.path.join(folder_path, "pointcloud.ply")
pointcloud_file_path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\CloudComPy\dataset\quality_good\cropped_pointcloud\pointcloud_copped_200.ply"
pointcloud_file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\cropped_pointcloud\pointcloud_copped_232.ply"
#pointcloud_file_path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\CloudComPy\dataset\new\pointcloud_copped_1.ply"



# pointcloud_normal_mesh_red(folder_path, pointcloud_file_path)
# pointcloud_green_mesh_red(folder_path, pointcloud_file_path)
pointcloud_gray_mesh_depthcolor(folder_path, pointcloud_file_path)
# pointcloud_red_mesh_normal(folder_path, pointcloud_file_path)
# mesh_only_green(folder_path, pointcloud_file_path)
