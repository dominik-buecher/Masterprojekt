import open3d as o3d
import numpy as np
import os

def get_distance_and_color(ply_file1, ply_file2, number_pointcloud, number_mesh):
    # Lade die Punktwolke der ersten Kugel aus der PLY-Datei
    point_cloud1 = o3d.io.read_point_cloud(ply_file1)

    # Extrahiere die Punkte aus der Punktwolke der ersten Kugel
    points1 = np.asarray(point_cloud1.points)

    # Berechne den Schwerpunkt (Mittelpunkt) der ersten Kugel
    center_of_sphere1 = np.mean(points1, axis=0)

    # Lade die Punktwolke der zweiten Kugel aus der PLY-Datei
    mesh_cloud = o3d.io.read_point_cloud(ply_file2)

    # Extrahiere die Punkte und Farben aus der Punktwolke der zweiten Kugel
    pointcloud_mesh_points = np.asarray(mesh_cloud.points)
    pointcloud_mesh_colors = np.asarray(mesh_cloud.colors)

    # Berechne den Abstand der Punkte der zweiten Kugel zum Schwerpunkt der ersten Kugel
    distances_to_center = np.linalg.norm(pointcloud_mesh_points - center_of_sphere1, axis=1)

    # Erstelle eine TXT-Datei und schreibe die Distanzen und RGB-Farbwerte hinein
    output_file = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\pointcloud_{number_pointcloud}_mesh_{number_mesh}.txt"  # Pfad zur Ausgabedatei

    with open(output_file, 'w') as f:
        for distance, color in zip(distances_to_center, pointcloud_mesh_colors):
            # Multipliziere die RGB-Werte mit 255, um sie in den Bereich von 0 bis 255 zu bringen
            r, g, b = color * 255
            # Schreibe die Distanz und die umgerechneten RGB-Werte in die Datei
            f.write(f"{distance} {int(r)} {int(g)} {int(b)}\n")

    print(f"Die Ausgabedatei wurde erfolgreich erstellt: {output_file}")

def process_ply_files(directory):
    # Erhalte eine Liste aller Dateien im Verzeichnis
    files = os.listdir(directory)
    
    # Filtere nach mesh und mesh_cloud Dateien
    mesh_files = [file for file in files if file.startswith("mesh_") and file.endswith(".ply")]
    mesh_cloud_files = [file for file in files if file.startswith("mesh_cloud") and file.endswith(".ply")]

    # Iteriere über die Paare von mesh und mesh_cloud Dateien
    for mesh_file in mesh_files:
        # Extrahiere die Nummer der aktuellen mesh-Datei
        mesh_number = mesh_file.split('_')[1].split('.')[0]
        # Finde die entsprechende mesh_cloud Datei
        mesh_cloud_file = f"mesh_cloud_{mesh_number}.ply"

        # Überprüfe, ob die entsprechende mesh_cloud Datei vorhanden ist
        if mesh_cloud_file in mesh_cloud_files:
            ply_file1 = os.path.join(directory, mesh_file)
            ply_file2 = os.path.join(directory, mesh_cloud_file)

            # Rufe die Funktion get_distance_and_color für das Paar auf
            get_distance_and_color(ply_file1, ply_file2, mesh_number, mesh_number)

# Definiere das Verzeichnis, in dem sich die PLY-Dateien befinden
directory = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data"

# Verarbeite die PLY-Dateien im Verzeichnis
process_ply_files(directory)
