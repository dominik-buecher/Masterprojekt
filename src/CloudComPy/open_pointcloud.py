import open3d as o3d
import numpy as np

def visualize_pointcloud(file_path, file_path2):
    # Einlesen der PointCloud
    mesh = o3d.io.read_point_cloud(file_path)
    cloud = o3d.io.read_point_cloud(file_path2)

    inline = mesh.paint_uniform_color([1, 0, 0])
    outline = cloud.paint_uniform_color([0, 1, 0])


    #o3d.visualization.draw_geometries([outline, mesh_circle, inline])
    o3d.visualization.draw_geometries([inline, outline])
        
    # # Visualisierung der PointCloud
    # o3d.visualization.draw_geometries([pcd],
    #                                   zoom=0.3412,
    #                                   front=[0.4257, -0.2125, -0.8795],
    #                                   lookat=[2.6172, 2.0475, 1.532],
    #                                   up=[-0.0694, -0.9768, 0.2024])


def visualize_pointcloud3(file_path_cloud, file_path_mesh1, file_path_mesh2, file_path_mesh3, file_path_mesh4, file_path_mesh5, file_path_mesh6, file_path_mesh7, file_path_mesh8):
    # Einlesen der PointCloud
    
    cloud = o3d.io.read_point_cloud(file_path_cloud)

    mesh1 = o3d.io.read_point_cloud(file_path_mesh1)
    mesh2 = o3d.io.read_point_cloud(file_path_mesh2)
    mesh3 = o3d.io.read_point_cloud(file_path_mesh3)
    mesh4 = o3d.io.read_point_cloud(file_path_mesh4)
    mesh5 = o3d.io.read_point_cloud(file_path_mesh5)
    mesh6 = o3d.io.read_point_cloud(file_path_mesh6)
    mesh7 = o3d.io.read_point_cloud(file_path_mesh7)
    mesh8 = o3d.io.read_point_cloud(file_path_mesh8)

    inline1 = mesh1.paint_uniform_color([1, 0, 0])
    inline2 = mesh2.paint_uniform_color([1, 0, 0])
    inline3 = mesh3.paint_uniform_color([1, 0, 0])
    inline4 = mesh4.paint_uniform_color([1, 0, 0])
    inline5 = mesh5.paint_uniform_color([1, 0, 0])
    inline6 = mesh6.paint_uniform_color([1, 0, 0])
    inline7 = mesh7.paint_uniform_color([1, 0, 0])
    inline8 = mesh8.paint_uniform_color([1, 0, 0])

    outline = cloud.paint_uniform_color([0, 1, 0])


    #o3d.visualization.draw_geometries([outline, mesh_circle, inline])
    o3d.visualization.draw_geometries([inline1, inline2, inline3, inline4, inline5, inline6, inline7, inline8, outline])
        
    # # Visualisierung der PointCloud
    # o3d.visualization.draw_geometries([pcd],
    #                                   zoom=0.3412,
    #                                   front=[0.4257, -0.2125, -0.8795],
    #                                   lookat=[2.6172, 2.0475, 1.532],
    #                                   up=[-0.0694, -0.9768, 0.2024])



def visualize_pointcloud1(file_path):
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
    crop_box = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-150, -150, -150), max_bound=(900, 900, 900))
    cropped_pcd = pcd.crop(crop_box)

    # Speichern der extrahierten Punktwolke in eine separate Datei
    o3d.io.write_point_cloud(r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\dataset\pointcloud\pointcloud_noBackround_cropped.ply", cropped_pcd)
    
    # Visualisierung der extrahierten Punktwolke
    o3d.visualization.draw_geometries([cropped_pcd],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])




def main():
    i = 1
    file_path1 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_{i}.ply"
    file_path2 = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_{i+1}.ply"
    file_path3 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_{i+2}.ply"
    file_path4 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_{i+3}.ply"
    file_path5 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_{i+4}.ply"
    file_path6 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_{i+5}.ply"
    file_path7 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_{i+6}.ply"
    file_path8 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_{i+7}.ply"

    # file_path1 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_1.ply"
    # file_path2 = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_7.ply"
    # file_path3 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_15.ply"
    # file_path4 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_18.ply"
    # file_path5 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_21.ply"
    # file_path6 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_22.ply"
    # file_path7 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_29.ply"
    # file_path8 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_30.ply"


## 75 einzeln und nah
## 100 viele und nah

    #file_path_cloud = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\pointcloud_1.ply"
    #path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\meshes\pointcloud_copped_1.ply\mesh_cloud\mesh_cloud_1.ply"
    path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\CloudComPy\dataset\quality_good\cropped_pointcloud\pointcloud_copped_1.ply"
    #path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\dataset\pointcloud\pointcloud_noBackround5_only_close.ply"
    file_path_cloud = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\cropped_pointcloud\pointcloud_copped_1.ply"
    file_path_cloud = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_1.ply\mesh_cloud\mesh_cloud_2.ply"
    #file_path2 = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data5\mesh_57.ply"
    #file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\pointcloud_kugel_frei.ply"
    #file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
    #file_path = "dataset/pointcloud_white_1_26_augmented.ply"
    #visualize_pointcloud(path, file_path_cloud)
    #crop_pointcloud(file_path)
    #visualize_pointcloud3(file_path_cloud, file_path1, file_path2, file_path3, file_path4, file_path5, file_path6, file_path7, file_path8)
    visualize_pointcloud1(path)

if __name__ == "__main__":
    main()
