import os
import sys
import math
import open3d as o3d
import numpy as np

import cloudComPy as cc
from cloudComPy import RANSAC_SD

# conda activate CloudComPy310
# cd C:\Users\domin\Documents\Studium\Master\CloudComPy310
# envCloudComPy.bat
# python C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\test2.py
#from gendata import dataDir, isCoordEqual



def is_majority_green(colors_matrix):
    
    colors_int = (colors_matrix * 255).astype(np.uint8)
    # Zähle die Anzahl der grünen Punkte
    green_count = 0
    
    # Iteriere durch jede Farbe in der Matrix
    for color in colors_int:
        r, g, b = color
        
        # Prüfe, ob die Grünkomponente größer ist als die anderen Komponenten
        if g > r and g > b:
            green_count += 1
    
    # Berechne den Anteil der grünen Punkte
    total_points = colors_int.shape[0]
    green_ratio = green_count / total_points
    
    # Prüfe, ob mehr als die Hälfte der Punkte grün sind
    if green_ratio > 0.5:
        return 1
    else:
        return 0



#pointcloud_file_path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\data\temp\pointcloud_cropped.ply"
#pointcloud_file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\cropped_kugel_frei.ply"
#pointcloud_file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data1\pointcloud_77.ply"
# pointcloud_file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\trauben_pointcloud.ply"
pointcloud_file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\cropped_pointcloud\pointcloud_copped_1.ply"

cloud = cc.loadPointCloud(pointcloud_file_path)
print("cloud: ", cloud)

pcd = o3d.io.read_point_cloud(pointcloud_file_path)


if cc.isPluginRANSAC_SD():
    print("RANSAC_SD is available!")
    
    # RANSAC-Parameter festlegen
    params = cc.RANSAC_SD.RansacParams()
    params.minSphereRadius = 5.0
    params.maxSphereRadius = 12.0
    params.supportPoints = 100
    params.allowSimplification = True   #-> Attempt to simplify shapes
    params.epsilon = 0.005              #-> Maximum distance if the samples to the ideal shape, default 0.005.
    params.bitmapEpsilon = 0.001        #-> Sampling resolution, should correspond to the average distance of neighbors points in data, default 0.001.
    params.optimizeForCloud(cloud)
    print("RANSAC_SD parameters: ", params)
    print(params.epsilon, params.bitmapEpsilon)
    print(params.minSphereRadius, params.maxSphereRadius)


    params.setPrimEnabled(cc.RANSAC_SD.RANSAC_PRIMITIVE_TYPES.RPT_SPHERE, True)
    params.setPrimEnabled(cc.RANSAC_SD.RANSAC_PRIMITIVE_TYPES.RPT_PLANE, False)
    params.setPrimEnabled(cc.RANSAC_SD.RANSAC_PRIMITIVE_TYPES.RPT_CYLINDER, False)
    params.setPrimEnabled(cc.RANSAC_SD.RANSAC_PRIMITIVE_TYPES.RPT_CONE, False)
    params.setPrimEnabled(cc.RANSAC_SD.RANSAC_PRIMITIVE_TYPES.RPT_TORUS, False)
    print("######################")
    meshes, clouds = cc.RANSAC_SD.computeRANSAC_SD(cloud, params)
    print("meshes: ", meshes)
    print("###################################################################################################")
    print("clouds: ", clouds)
    ret = cc.SavePointCloud(cloud, rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\pointcloud.ply")
    #base_path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\Dominik\CloudComPy"
    base_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy"
    for i, mesh in enumerate(meshes):
        if mesh is not None and mesh.isA(cc.CC_TYPES.SPHERE): #CYLINDER
            print("Radius: ", mesh.getRadius())
            print("Geschafft:       ###################################################################################")
            # sphere_cloud_name = f"\sphere_{i + 1}.ply"
            # # sphere_cloud_path = os.path.join(base_path, sphere_cloud_name)
            # print(f"Saving sphere {i + 1} to {sphere_cloud_path}")
            #mesh.saveToFile(sphere_cloud_path)
            if mesh.getRadius() < 20.0 and mesh.getRadius() > 5.0:
                path = rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\temp\mesh_cloud_{i + 1}.ply"
                ret = cc.SavePointCloud(clouds[i], path)
                temp = o3d.io.read_point_cloud(path)


                colors_float = np.asarray(temp.colors)
                #green = is_majority_green(colors_float)

                #if green == 1:
                ret = cc.SaveMesh(mesh, rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_{i + 1}.ply")

                mesh_cloud = mesh.getAssociatedCloud()
                ret = cc.SavePointCloud(clouds[i], rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\cloud_mesh_{i + 1}.ply")
                


# Speichern der ursprünglichen Cloud mit den gefundenen Formen
shapes = [cloud]
for m in meshes:
    if m is not None:
        shapes.append(m)

# cc.SavePointCloud(cloud, rf"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\dataSample.ply")

cc.SaveEntities(shapes, os.path.join(base_path, "ransac.pcd"))
#cc.SaveEntities(shapes, r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\Dominik\CloudComPy\dataSample.ply")