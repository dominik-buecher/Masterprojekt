import open3d as o3d
import open3d as o3d
from sklearn.linear_model import RANSACRegressor
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from joblib import dump, load

# pcd = o3d.io.read_point_cloud("data/pointcloud_white_3_46.ply")
# plane_model, inliers = pcd.segment_plane(distance_threshold=250, ransac_n=4, num_iterations=10000)


# inlier_cloud = pcd.select_by_index(inliers)
# outlier_cloud = pcd.select_by_index(inliers, invert=True)

# inlier_cloud.paint_uniform_color([1, 0, 0])
# outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])

# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])



def extract_grapes(pointcloud_file):

    pointcloud = o3d.io.read_point_cloud(pointcloud_file)
    plane_model, inliers = pointcloud.segment_plane(distance_threshold=250, ransac_n=4, num_iterations=10000)
    inlier_cloud = pointcloud.select_by_index(inliers)

    points = np.asarray(inlier_cloud.points)
    colors = np.asarray(inlier_cloud.colors)

    # points = np.asarray(pointcloud.points)
    # colors = np.asarray(pointcloud.colors)
    
    grape_pointcloud1 = o3d.geometry.PointCloud()
    grape_pointcloud1.points = o3d.utility.Vector3dVector(points)
    grape_pointcloud1.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([grape_pointcloud1])
    o3d.io.write_point_cloud("data/trauben_pointcloud.ply", grape_pointcloud1)


    # Invertiere die y-Koordinaten
    points[:, 1:] *= -1
    print("points[:, 2]: ",points[:, 2])
    print("min(points[:, 2]): ", min(points[:, 2]))
    var = (min(points[:, 2])* -1) - 1000
    # Bereich einschränken
    front_points = points[points[:, 2] > min(points[:, 2]) + var]
    front_colors = colors[points[:, 2] > min(points[:, 2]) + var]

    # RANSAC
    ransac = RANSACRegressor()
    ransac.fit(front_points[:, :2], front_points[:, 2])
    epsilon = 800  # => Schwelle für die Einbeziehung von Punkten als Inliers

    dump(ransac, 'model/ransac_model.joblib')

    # Lade den RANSAC-Algorithmus aus der Datei
    #loaded_ransac = load('ransac_model.joblib')

    # Filtere Punkte basierend auf RANSAC
    inlier_mask = (
        (front_points[:, 2] > ransac.predict(front_points[:, :2]) - epsilon) &
        (front_points[:, 2] < ransac.predict(front_points[:, :2]) + epsilon)
    )

    grape_points = front_points[inlier_mask]
    grape_colors = front_colors[inlier_mask]

    grape_pointcloud = o3d.geometry.PointCloud()
    grape_pointcloud.points = o3d.utility.Vector3dVector(grape_points)
    grape_pointcloud.colors = o3d.utility.Vector3dVector(grape_colors)

    o3d.visualization.draw_geometries([grape_pointcloud])
    o3d.io.write_point_cloud("data/trauben_pointcloud.ply", grape_pointcloud)


if __name__ == "__main__":
    #pointcloud_file = "data/pointcloud_white_5_83.ply"
    #pointcloud_file = "data/pointcloud_black_4_45.ply"
    #pointcloud_file = "data/pointcloud_white_1_45.ply"
    pointcloud_file = "data/pointcloud_white_2_16.ply"
    #pointcloud_file = "data/pointcloud_white_3_75.ply"
    pointcloud_file = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
    extract_grapes(pointcloud_file)
    # crop_pointcloud(pointcloud_file)

