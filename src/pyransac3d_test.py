# import pyransac3d as pyrsc

# points = load_points(.) # Load your point cloud as a numpy array (N, 3)
# sph = pyrsc.Sphere()
# center, radius, inliers = sph.fit(points, thresh=0.4)

#import cloudComPy as cc
 
import sys
import numpy as np
import open3d as o3d

sys.path.append(".")
import pyransac3d as pyran

# mesh_in = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
# vertices = np.asarray(mesh_in.vertices)
# noise = 0.5
# vertices += np.random.logistic(0, noise, size=vertices.shape)
# mesh_in.vertices = o3d.utility.Vector3dVector(vertices)
# mesh_in.compute_vertex_normals()
#mesh_in.paint_uniform_color([0.1, 0.9, 0.1])


sph = pyran.Sphere()
#sph = pyran.Circle()
#sph = pyran.Cylinder()

#pointcloud_file = "data/pointcloud_white_3_75.ply"
#pointcloud_file = "data/temp/pointcloud_cropped.ply"
pointcloud_file = "data/cropped_kugel_frei.ply"
pointcloud = o3d.io.read_point_cloud(pointcloud_file)

points = np.asarray(pointcloud.points)

center, radius, inliers = sph.fit(points, thresh=0.4, maxIteration=2000)
print("center: " + str(center))
print("radius: " + str(radius))


inline = pointcloud.select_by_index(inliers).paint_uniform_color([1, 0, 0])
outline = pointcloud.select_by_index(inliers, invert=True).paint_uniform_color([0, 1, 0])

mesh_circle = o3d.geometry.TriangleMesh.create_sphere(radius=5.0)
mesh_circle.compute_vertex_normals()
mesh_circle.paint_uniform_color([0.9, 0.1, 0.1])
mesh_circle = mesh_circle.translate((center[0], center[1], center[2]))
#o3d.visualization.draw_geometries([outline, mesh_circle, inline])
o3d.visualization.draw_geometries([inline, outline])
#o3d.visualization.draw_geometries([inline])
# # o3d.visualization.draw_geometries([mesh_circle, outline])


# inline_test = pointcloud.select_by_index(inliers)


# # Speichern Sie die PointCloud im PLY-Format
# file_path = fr"data\temp\pointcloud_cropped.ply"
# o3d.io.write_point_cloud(file_path, inline_test)


# pointcloud = o3d.io.read_point_cloud(file_path)

# points = np.asarray(pointcloud.points)
# o3d.visualization.draw_geometries([pointcloud])
# center, radius, inliers = sph.fit(points, thresh=4.0, maxIteration=2000)
# print("center: " + str(center))
# print("radius: " + str(radius))



# inline = pointcloud.select_by_index(inliers).paint_uniform_color([1, 0, 0])
# outline = pointcloud.select_by_index(inliers, invert=True).paint_uniform_color([0, 1, 0])


# o3d.visualization.draw_geometries([inline, outline])





# import sys
# import numpy as np
# import open3d as o3d

# sys.path.append(".")
# import pyransac3d as pyran

# # Laden Sie die Punktwolke
# pointcloud_file = "data/pointcloud_white_3_75.ply"
# pointcloud = o3d.io.read_point_cloud(pointcloud_file)
# points = np.asarray(pointcloud.points)

# # Fügen Sie eine Rauschunterdrückung hinzu, um kleinere Details zu glätten
# points_smoothed = o3d.geometry.PointCloud()
# points_smoothed.points = o3d.utility.Vector3dVector(points)
# points_smoothed.estimate_normals()
# points_smoothed = points_smoothed.voxel_down_sample(voxel_size=0.01)

# # Passen Sie den Code zur Kugelfit an
# sph = pyran.Sphere()
# center, radius, inliers = sph.fit(np.asarray(points_smoothed.points), thresh=0.1, maxIteration = 1500)  # Reduzieren Sie den Schwellenwert

# # Zeigen Sie das Ergebnis an
# inline = pointcloud.select_by_index(inliers).paint_uniform_color([1, 0, 0])
# outline = pointcloud.select_by_index(inliers, invert=True).paint_uniform_color([0, 1, 0])

# mesh_circle = o3d.geometry.TriangleMesh.create_sphere(radius=5.0)
# mesh_circle.compute_vertex_normals()
# mesh_circle.paint_uniform_color([0.9, 0.1, 0.1])
# mesh_circle = mesh_circle.translate((center[0], center[1], center[2]))

# o3d.visualization.draw_geometries([outline, mesh_circle, inline])
