import open3d as o3d
import numpy as np
import os

# This function calculates the distance from the mesh_cloud points to the mesh sphere
# Additionally, it calculates spherical coordinates (d, theta, phi)
# It creates a txt file with the distance, spherical coordinates, and the RGB values of the point
def calc_distance_good_quality(mesh_path, mesh_cloud_path, output_file_path, i):
    # Create the output directory if it doesn't exist
    os.makedirs(output_file_path, exist_ok=True)

    # Load the point cloud of the sphere from the PLY file
    ply_files = [f for f in os.listdir(mesh_cloud_path) if f.endswith('.ply')]
    for ply_file in ply_files:
        mesh_cloud_file = os.path.join(mesh_cloud_path, ply_file)
        mesh_file = os.path.join(mesh_path, f'mesh_{ply_file.split("_")[-1]}')
        
        # Remove .ply extension from the filename
        file_suffix = ply_file.split("_")[-1].replace('.ply', '')
        output_file = os.path.join(output_file_path, f'distance_{i}_{file_suffix}.txt')

        # Load the point cloud of the first sphere from the PLY file
        point_cloud = o3d.io.read_point_cloud(mesh_file)
        points = np.asarray(point_cloud.points)
        center_of_sphere = np.mean(points, axis=0)

        # Load the point cloud of the second sphere from the PLY file
        mesh_cloud = o3d.io.read_point_cloud(mesh_cloud_file)
        pointcloud_mesh_points = np.asarray(mesh_cloud.points)
        pointcloud_mesh_colors = np.asarray(mesh_cloud.colors)

        # Calculate the distance of the points of the second sphere to the center of the first sphere
        distances_to_center = np.linalg.norm(pointcloud_mesh_points - center_of_sphere, axis=1)

        # Calculate spherical coordinates
        relative_points = pointcloud_mesh_points - center_of_sphere
        d = distances_to_center
        theta = np.arctan2(relative_points[:, 1], relative_points[:, 0])
        phi = np.arccos(relative_points[:, 2] / d)

        # Create a TXT file and write the distances, spherical coordinates, and RGB color values
        with open(output_file, 'w') as f:
            for distance, t, p, color in zip(d, theta, phi, pointcloud_mesh_colors):
                r, g, b = color * 255
                f.write(f"{distance} {t} {p} {int(r)} {int(g)} {int(b)}\n")

        print(f"Output file successfully created: {output_file}")

        # Create a sphere geometry for the center of the first sphere
        center_point = o3d.geometry.PointCloud()
        center_point.points = o3d.utility.Vector3dVector([center_of_sphere])

        # Color the center point red
        center_point.paint_uniform_color([1, 0, 0])  # Red
        mesh_cloud.paint_uniform_color([1, 0, 0])  # Red

def calc_distance_bad_quality(mesh_path, mesh_cloud_path, output_file_path, i):
    # Create the output directory if it doesn't exist
    os.makedirs(output_file_path, exist_ok=True)

    # Load the point cloud of the sphere from the PLY file
    ply_files = [f for f in os.listdir(mesh_cloud_path) if f.endswith('.ply')]
    for ply_file in ply_files:
        mesh_cloud_file = os.path.join(mesh_cloud_path, ply_file)
        mesh_file = os.path.join(mesh_path, f'mesh_{ply_file.split("_")[-1]}')
        
        # Remove .ply extension from the filename
        file_suffix = ply_file.split("_")[-1].replace('.ply', '')
        output_file = os.path.join(output_file_path, f'distance_{i}_{file_suffix}.txt')

        # Load the point cloud of the first sphere from the PLY file
        point_cloud = o3d.io.read_point_cloud(mesh_file)
        points = np.asarray(point_cloud.points)
        center_of_sphere = np.mean(points, axis=0)

        # Load the point cloud of the second sphere from the PLY file
        mesh_cloud = o3d.io.read_point_cloud(mesh_cloud_file)
        pointcloud_mesh_points = np.asarray(mesh_cloud.points)
        pointcloud_mesh_colors = np.asarray(mesh_cloud.colors)

        # Calculate the distance of the points of the second sphere to the center of the first sphere
        distances_to_center = np.linalg.norm(pointcloud_mesh_points - center_of_sphere, axis=1)

        # Calculate spherical coordinates
        relative_points = pointcloud_mesh_points - center_of_sphere
        d = distances_to_center
        theta = np.arctan2(relative_points[:, 1], relative_points[:, 0])
        phi = np.arccos(relative_points[:, 2] / d)

        # Create a TXT file and write the distances, spherical coordinates, and RGB color values
        with open(output_file, 'w') as f:
            for distance, t, p, color in zip(d, theta, phi, pointcloud_mesh_colors):
                r, g, b = color * 255
                f.write(f"{distance} {t} {p} {int(r)} {int(g)} {int(b)}\n")

        print(f"Output file successfully created: {output_file}")

        # Create a sphere geometry for the center of the first sphere
        center_point = o3d.geometry.PointCloud()
        center_point.points = o3d.utility.Vector3dVector([center_of_sphere])

        # Color the center point red
        center_point.paint_uniform_color([1, 0, 0])  # Red
        mesh_cloud.paint_uniform_color([1, 0, 0])  # Red

# Execute for good quality datasets
for i in range(1, 101):
    output_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\distance"
    mesh_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh"
    mesh_cloud_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color_green"
    calc_distance_good_quality(mesh_path, mesh_cloud_path, output_file_path, i)

# Execute for bad quality datasets
for i in range(1, 61):
    output_file_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\distance"
    mesh_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\meshes\pointcloud_copped_{i}.ply\mesh"
    mesh_cloud_path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color"
    calc_distance_bad_quality(mesh_path, mesh_cloud_path, output_file_path, i)
