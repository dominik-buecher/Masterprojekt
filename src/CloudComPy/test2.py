import os
import sys
import math

#from gendata import dataDir, isCoordEqual
import cloudComPy as cc
from cloudComPy import RANSAC_SD

#pointcloud_file_path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\data\temp\pointcloud_cropped.ply"
pointcloud_file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\cropped_kugel_frei.ply"
cloud = cc.loadPointCloud(pointcloud_file_path)
print("cloud: ", cloud)

if cc.isPluginRANSAC_SD():
    print("RANSAC_SD is available!")
    
    # RANSAC-Parameter festlegen
    params = cc.RANSAC_SD.RansacParams()
    params.maxSphereRadius = 3
    params.minSphereRadius = 0.5

    params.optimizeForCloud(cloud)
    print("RANSAC_SD parameters: ", params)

    meshes, clouds = cc.RANSAC_SD.computeRANSAC_SD(cloud, params)
    print("meshes: ", meshes)
    print("clouds: ", clouds)
    #base_path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\Dominik\CloudComPy"
    base_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\Dominik\CloudComPy"
    for i, mesh in enumerate(meshes):
        if mesh is not None and mesh.isA(cc.CC_TYPES.SPHERE):
 
            sphere_cloud_name = f"sphere_{i + 1}.ply"
            sphere_cloud_path = os.path.join(base_path, sphere_cloud_name)
            print(f"Saving sphere {i + 1} to {sphere_cloud_path}")
            #mesh.saveToFile(sphere_cloud_path)
            ret = cc.SavePointCloud(cloud, rf"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\Dominik\CloudComPy\dataSample_{i + 1}.ply")
            ret = cc.SavePointCloud(mesh, rf"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\Dominik\CloudComPy\dataSample_{i + 1}.ply")

# Speichern der ursprünglichen Cloud mit den gefundenen Formen
shapes = [cloud]
for m in meshes:
    if m is not None:
        shapes.append(m)

cc.SavePointCloud(cloud, rf"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\Dominik\CloudComPy\dataSample.ply")

cc.SaveEntities(shapes, os.path.join(base_path, "ransac.pcd"))
#cc.SaveEntities(shapes, r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\Dominik\CloudComPy\dataSample.ply")