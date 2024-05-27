import open3d as o3d
import numpy as np
import os

# This function calculates the distance from the mash_cloud points to the mesh sphere
# The higher the Distance the worse is the mesh fittet on the sphere
# In the End it creates an txt file with the distance and the rgb values of this point
def calc_distance_good_quality(mesh_path, mesh_cloud_path, output_file_path, i):
    # Erstelle den Ausgangsordner, falls er nicht existiert
    os.makedirs(output_file_path, exist_ok=True)

    # Lade die Punktwolke der Kugel aus der PLY-Datei
    ply_files = [f for f in os.listdir(mesh_cloud_path) if f.endswith('.ply')]
    for ply_file in ply_files:
        mesh_cloud_file = os.path.join(mesh_cloud_path, ply_file)
        mesh_file = os.path.join(mesh_path, f'mesh_{ply_file.split("_")[-1]}')
        output_file = os.path.join(output_file_path, f'distance_{i}_{ply_file.split("_")[-1]}.txt')

        # Lade die Punktwolke der ersten Kugel aus der PLY-Datei
        point_cloud = o3d.io.read_point_cloud(mesh_file)
        points = np.asarray(point_cloud.points)
        center_of_sphere = np.mean(points, axis=0)

        # Lade die Punktwolke der zweiten Kugel aus der PLY-Datei
        mesh_cloud = o3d.io.read_point_cloud(mesh_cloud_file)
        pointcloud_mesh_points = np.asarray(mesh_cloud.points)
        pointcloud_mesh_colors = np.asarray(mesh_cloud.colors)

        # Berechne den Abstand der Punkte der zweiten Kugel zum Schwerpunkt der ersten Kugel
        distances_to_center = np.linalg.norm(pointcloud_mesh_points - center_of_sphere, axis=1)

        # Erstelle eine TXT-Datei und schreibe die Distanzen und RGB-Farbwerte hinein
        with open(output_file, 'w') as f:
            for distance, color in zip(distances_to_center, pointcloud_mesh_colors):
                r, g, b = color * 255
                f.write(f"{distance} {int(r)} {int(g)} {int(b)}\n")

        print(f"Die Ausgabedatei wurde erfolgreich erstellt: {output_file}")

        # Erstelle eine Kugelgeometrie für den Mittelpunkt der ersten Kugel
        center_point = o3d.geometry.PointCloud()
        center_point.points = o3d.utility.Vector3dVector([center_of_sphere])

        # Färbe den Mittelpunkt rot
        center_point.paint_uniform_color([1, 0, 0])  # Rot
        mesh_cloud.paint_uniform_color([1, 0, 0])  # Rot
        # Visualisierung der Punktwolken zusammen mit dem roten Mittelpunkt
        #o3d.visualization.draw_geometries([point_cloud, mesh_cloud, center_point])





def calc_distance_bad_quality(mesh_path, mesh_cloud_path, output_file_path, i):
    # Erstelle den Ausgangsordner, falls er nicht existiert
    os.makedirs(output_file_path, exist_ok=True)

    # Lade die Punktwolke der Kugel aus der PLY-Datei
    ply_files = [f for f in os.listdir(mesh_cloud_path) if f.endswith('.ply')]
    for ply_file in ply_files:
        mesh_cloud_file = os.path.join(mesh_cloud_path, ply_file)
        mesh_file = os.path.join(mesh_path, f'mesh_{ply_file.split("_")[-1]}')
        output_file = os.path.join(output_file_path, f'distance_{i}_{ply_file.split("_")[-1]}.txt')

        # Lade die Punktwolke der ersten Kugel aus der PLY-Datei
        point_cloud = o3d.io.read_point_cloud(mesh_file)
        points = np.asarray(point_cloud.points)
        center_of_sphere = np.mean(points, axis=0)

        # Lade die Punktwolke der zweiten Kugel aus der PLY-Datei
        mesh_cloud = o3d.io.read_point_cloud(mesh_cloud_file)
        pointcloud_mesh_points = np.asarray(mesh_cloud.points)
        pointcloud_mesh_colors = np.asarray(mesh_cloud.colors)

        # Berechne den Abstand der Punkte der zweiten Kugel zum Schwerpunkt der ersten Kugel
        distances_to_center = np.linalg.norm(pointcloud_mesh_points - center_of_sphere, axis=1)

        # Erstelle eine TXT-Datei und schreibe die Distanzen und RGB-Farbwerte hinein
        with open(output_file, 'w') as f:
            for distance, color in zip(distances_to_center, pointcloud_mesh_colors):
                r, g, b = color * 255
                f.write(f"{distance} {int(r)} {int(g)} {int(b)}\n")

        print(f"Die Ausgabedatei wurde erfolgreich erstellt: {output_file}")

        # Erstelle eine Kugelgeometrie für den Mittelpunkt der ersten Kugel
        center_point = o3d.geometry.PointCloud()
        center_point.points = o3d.utility.Vector3dVector([center_of_sphere])

        # Färbe den Mittelpunkt rot
        center_point.paint_uniform_color([1, 0, 0])  # Rot
        mesh_cloud.paint_uniform_color([1, 0, 0])  # Rot
        # Visualisierung der Punktwolken zusammen mit dem roten Mittelpunkt
        #o3d.visualization.draw_geometries([point_cloud, mesh_cloud, center_point])



for i in range(1, 101):
    #output_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color_green\distance"  # Pfad zur Ausgabedatei
    output_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\distance"  # Pfad zur Ausgabedatei
    mesh_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh"
    mesh_cloud_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color_green"
    calc_distance_good_quality(mesh_path, mesh_cloud_path, output_file_path, i)





for i in range(1, 61):
    #output_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color\distance"  # Pfad zur Ausgabedatei
    output_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\distance"  # Pfad zur Ausgabedatei
    
    mesh_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\meshes\pointcloud_copped_{i}.ply\mesh"
    mesh_cloud_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color"
    calc_distance_bad_quality(mesh_path, mesh_cloud_path, output_file_path, i)