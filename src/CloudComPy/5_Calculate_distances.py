import open3d as o3d
import numpy as np
import os

def calculate_and_save_data(mesh_file, mesh_cloud_file, output_file, spherical=False):
    # Lade die Punktwolke der Referenzkugel aus der PLY-Datei
    point_cloud = o3d.io.read_point_cloud(mesh_file)
    points = np.asarray(point_cloud.points)
    center_of_sphere = np.mean(points, axis=0)

    # Lade die Punktwolke der zu vergleichenden Kugel aus der PLY-Datei
    mesh_cloud = o3d.io.read_point_cloud(mesh_cloud_file)
    pointcloud_mesh_points = np.asarray(mesh_cloud.points)
    pointcloud_mesh_colors = np.asarray(mesh_cloud.colors)

    # Berechne den Abstand der Punkte zur Mitte der Referenzkugel
    distances_to_center = np.linalg.norm(pointcloud_mesh_points - center_of_sphere, axis=1)

    if spherical:
        # Berechne Kugelkoordinaten
        relative_points = pointcloud_mesh_points - center_of_sphere
        theta = np.arctan2(relative_points[:, 1], relative_points[:, 0])
        phi = np.arccos(relative_points[:, 2] / distances_to_center)

        # Speichern in einer TXT-Datei mit Kugelkoordinaten
        with open(output_file, 'w') as f:
            for distance, t, p, color in zip(distances_to_center, theta, phi, pointcloud_mesh_colors):
                r, g, b = color * 255
                f.write(f"{distance} {t} {p} {int(r)} {int(g)} {int(b)}\n")
    else:
        # Speichern in einer TXT-Datei mit kartesischen Koordinaten
        with open(output_file, 'w') as f:
            for distance, color in zip(distances_to_center, pointcloud_mesh_colors):
                r, g, b = color * 255
                f.write(f"{distance} {int(r)} {int(g)} {int(b)}\n")

    print(f"Output file successfully created: {output_file}")

def process_quality_data(mesh_path, mesh_cloud_path, output_file_path, i, quality, spherical=False):
    # Erstelle den Ausgangsordner, falls er nicht existiert
    os.makedirs(output_file_path, exist_ok=True)

    # Lade die Punktwolken-Dateien
    ply_files = [f for f in os.listdir(mesh_cloud_path) if f.endswith('.ply')]
    for ply_file in ply_files:
        mesh_cloud_file = os.path.join(mesh_cloud_path, ply_file)
        mesh_file = os.path.join(mesh_path, f'mesh_{ply_file.split("_")[-1]}')

        # Entferne .ply-Erweiterung vom Dateinamen
        file_suffix = ply_file.split("_")[-1].replace('.ply', '')
        
        coord_type = 'spherical' if spherical else 'cartesian'
        output_file = os.path.join(output_file_path, f'{coord_type}\{quality}_{coord_type}_distance_{i}_{file_suffix}.txt')

        # Berechne Distanzen und speichere die Ergebnisse
        calculate_and_save_data(mesh_file, mesh_cloud_file, output_file, spherical=spherical)

# Hauptprozess für gute Qualität
for i in range(1, 251):
    output_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\distance"
    mesh_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh"
    mesh_cloud_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color_green"
    
    process_quality_data(mesh_path, mesh_cloud_path, output_file_path, i, quality='good', spherical=False)
    process_quality_data(mesh_path, mesh_cloud_path, output_file_path, i, quality='good', spherical=True)

# Hauptprozess für schlechte Qualität
for i in range(1, 61):
    output_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\distance"
    mesh_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\meshes\pointcloud_copped_{i}.ply\mesh"
    mesh_cloud_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color"
    
    process_quality_data(mesh_path, mesh_cloud_path, output_file_path, i, quality='bad', spherical=False)
    process_quality_data(mesh_path, mesh_cloud_path, output_file_path, i, quality='bad', spherical=True)
