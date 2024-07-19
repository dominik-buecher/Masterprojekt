import os
import sys
import math
import open3d as o3d
import numpy as np
import cloudComPy as cc
from cloudComPy import RANSAC_SD
import shutil


# conda activate CloudComPy310
# cd C:\Users\domin\Documents\Studium\Master\CloudComPy310
# cd C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\CloudComPy310
# envCloudComPy.bat
# python C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\test2.py
# python C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\CloudComPy\2_RANSAC_shape_detection_single.py
#from gendata import dataDir, isCoordEqual



# Definieren des Ordners mit Pointcloud-Dateien
folder_path = r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\CloudComPy\dataset\new"

# Schleife, um alle Pointcloud-Dateien im Ordner zu verarbeiten
for filename in os.listdir(folder_path):
    # Überprüfen, ob die Datei eine .ply-Datei ist
    if filename.endswith('.ply'):
        pointcloud_file_path = os.path.join(folder_path, filename)
        print("filename: ", filename)
        # Laden der Pointcloud
        cloud = cc.loadPointCloud(pointcloud_file_path)
        print("cloud: ", cloud)

        if cc.isPluginRANSAC_SD():
            print("RANSAC_SD is available!")
            
            # Konfigurieren der RANSAC-Parameter
            params = cc.RANSAC_SD.RansacParams()
            # params.minSphereRadius = 5.0
            # params.maxSphereRadius = 15.0
            # params.supportPoints = 100
            # params.allowSimplification = True
            # params.epsilon = 0.005
            # params.bitmapEpsilon = 0.001
            # params.optimizeForCloud(cloud)
            print("RANSAC_SD parameters: ", params)
            
            # Ausführen der RANSAC-Shape-Erkennung
            # params.setPrimEnabled(cc.RANSAC_SD.RANSAC_PRIMITIVE_TYPES.RPT_SPHERE, True)
            # params.setPrimEnabled(cc.RANSAC_SD.RANSAC_PRIMITIVE_TYPES.RPT_PLANE, False)
            # params.setPrimEnabled(cc.RANSAC_SD.RANSAC_PRIMITIVE_TYPES.RPT_CYLINDER, False)
            # params.setPrimEnabled(cc.RANSAC_SD.RANSAC_PRIMITIVE_TYPES.RPT_CONE, False)
            # params.setPrimEnabled(cc.RANSAC_SD.RANSAC_PRIMITIVE_TYPES.RPT_TORUS, False)
            
            meshes, clouds = cc.RANSAC_SD.computeRANSAC_SD(cloud, params)
            print("meshes: ", meshes)
            print("clouds: ", clouds)

            # Speichern und Verarbeiten der Shapes
            base_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy"
            
            # Pfade für temporäre Dateien und für die Speicherung der Meshes und Mesh-Clouds
            temp_path = os.path.join(base_path, "tmp")
            mesh_base_path = os.path.join(base_path, "dataset/quality_bad/meshes")

            # Erstellen Sie die Ordner, falls sie noch nicht existieren
            os.makedirs(temp_path, exist_ok=True)
            os.makedirs(mesh_base_path, exist_ok=True)


            for i, mesh in enumerate(meshes):
                if mesh is not None and mesh.isA(cc.CC_TYPES.SPHERE):
                    print("Radius: ", mesh.getRadius())
                    print("Processing sphere ", i + 1)

                    # Speichern der Datei, wenn sie den Anforderungen entspricht
                    #if 5.0 <= mesh.getRadius() <= 15.0:
                    mesh_dir = os.path.join(mesh_base_path, filename, "mesh")
                    mesh_cloud_dir = os.path.join(mesh_base_path, filename, "mesh_cloud")

                    # Erstellen Sie die Verzeichnisse, falls sie noch nicht existieren
                    os.makedirs(mesh_dir, exist_ok=True)
                    os.makedirs(mesh_cloud_dir, exist_ok=True)

                    # Speichern des Meshs
                    mesh_path = f"C:/Users/Dominik/Documents/Studium/Master/Masterprojekt/Masterprojekt/src/CloudComPy/dataset/new/mesh/mesh_{i + 1}.ply"
                    print("mesh_path: ", mesh_path)
                    cc.SaveMesh(mesh, mesh_path)
                    mesh_cloud_path = f"C:/Users/Dominik/Documents/Studium/Master/Masterprojekt/Masterprojekt/src/CloudComPy/dataset/new/mesh_cloud/mesh_cloud_{i + 1}.ply"
                    print("mesh_cloud_path: ", mesh_cloud_path)
                    cc.SavePointCloud(clouds[i], mesh_cloud_path)

        print(f"Finished processing pointcloud file: {filename}")

print("Processing complete.")
shutil.rmtree(temp_path)
