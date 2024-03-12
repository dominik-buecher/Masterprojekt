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
    


def crop_pointcloud(file_path):
    # Einlesen der PointCloud
    pcd = o3d.io.read_point_cloud(file_path)

    # Filtern der Punktwolke, um einen bestimmten Bereich zu extrahieren
    # Beispiel: Nur Punkte innerhalb eines bestimmten Bereichs um den Ursprung behalten
    voxel_size = 0.05
    crop_box = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-150, -150, -150), max_bound=(600, 600, 600))
    cropped_pcd = pcd.crop(crop_box)

    # Speichern der extrahierten Punktwolke in eine separate Datei
    o3d.io.write_point_cloud("data/cropped_kugel_frei.ply", cropped_pcd)

    # Visualisierung der extrahierten Punktwolke
    o3d.visualization.draw_geometries([cropped_pcd],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])




def main():

    file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\pointcloud_kugel_frei.ply"
    #file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
    #file_path = "dataset/pointcloud_white_1_26_augmented.ply"
    #visualize_pointcloud(file_path)
    crop_pointcloud(file_path)


if __name__ == "__main__":
    main()
