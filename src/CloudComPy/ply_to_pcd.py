import open3d as o3d

input_path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\CloudComPy\dataset\quality_good\cropped_pointcloud\pointcloud_copped_1.ply"
# Lade die .ply Punktwolke
ply_point_cloud = o3d.io.read_point_cloud(input_path)

# Speichere die Punktwolke im .pcd Format
o3d.io.write_point_cloud(r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\CloudComPy\dataset\pointcloud_cropped.pcd", ply_point_cloud)

print("Punktwolke erfolgreich von .ply zu .pcd konvertiert")
