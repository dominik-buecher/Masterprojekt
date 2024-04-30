import open3d as o3d
import numpy as np

# Lade die Punktwolke der ersten Kugel aus der PLY-Datei
ply_file1 = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_1.ply"  # Ersetze durch den Pfad deiner PLY-Datei
point_cloud1 = o3d.io.read_point_cloud(ply_file1)

# Extrahiere die Punkte aus der Punktwolke der ersten Kugel
points1 = np.asarray(point_cloud1.points)

# Berechne den Schwerpunkt (Mittelpunkt) der ersten Kugel
center_of_sphere1 = np.mean(points1, axis=0)

# Gib den Mittelpunkt der ersten Kugel aus
print(f"Der Mittelpunkt der ersten Kugel ist: {center_of_sphere1}")

# Lade die Punktwolke der zweiten Kugel aus der PLY-Datei
ply_file2 = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_1.ply"  # Ersetze durch den Pfad deiner zweiten PLY-Datei
mesh_cloud = o3d.io.read_point_cloud(ply_file2)

# Extrahiere die Punkte aus der Punktwolke der zweiten Kugel
pointcloud_mesh_points = np.asarray(mesh_cloud.points)
pointcloud_mesh_colors = np.asarray(mesh_cloud.colors)

# Berechne den Abstand der Punkte der zweiten Kugel zum Schwerpunkt der ersten Kugel
distances_to_center = np.linalg.norm(pointcloud_mesh_points - center_of_sphere1, axis=1)

# Gib die Abstände aus
print("Die Abstände der Punkte der zweiten Kugel zum Schwerpunkt der ersten Kugel sind:")
print(distances_to_center)
print("##############################")
print("pointcloud_mesh_colors: ", pointcloud_mesh_colors)
# Du kannst die Abstände auch in einer Datei speichern oder weiterverarbeiten
# Zum Beispiel, um sie in einer Textdatei zu speichern:
np.savetxt("distances_to_center.txt", distances_to_center)


# Erstelle eine TXT-Datei und schreibe die Distanzen und RGB-Farbwerte hinein
output_file = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\output.txt"  # Pfad zur Ausgabedatei

with open(output_file, 'w') as f:
    for distance, color in zip(distances_to_center, pointcloud_mesh_colors):
        # Multipliziere die RGB-Werte mit 255, um sie in den Bereich von 0 bis 255 zu bringen
        r, g, b = color * 255
        # Schreibe die Distanz und die umgerechneten RGB-Werte in die Datei
        f.write(f"{distance} {int(r)} {int(g)} {int(b)}\n")

print(f"Die Ausgabedatei wurde erfolgreich erstellt: {output_file}")

# Erstelle eine Kugelgeometrie für den Mittelpunkt der ersten Kugel
center_point = o3d.geometry.PointCloud()
center_point.points = o3d.utility.Vector3dVector([center_of_sphere1])

# Färbe den Mittelpunkt rot
center_point.paint_uniform_color([1, 0, 0])  # Rot
mesh_cloud.paint_uniform_color([1, 0, 0])  # Rot
# Visualisierung der Punktwolken zusammen mit dem roten Mittelpunkt
o3d.visualization.draw_geometries([point_cloud1, mesh_cloud, center_point])
