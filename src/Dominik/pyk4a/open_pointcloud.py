import open3d as o3d
import numpy as np

def visualize_pointcloud(file_path):
    # Einlesen der PointCloud
    pcd = o3d.io.read_point_cloud(file_path)

    # Visualisierung der PointCloud
    o3d.visualization.draw_geometries([pcd],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])



def main():

    file_path = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\dataset\pointcloud\pointcloud_cameraPos-0_white_2.ply"

    visualize_pointcloud(file_path)


if __name__ == "__main__":
    main()
