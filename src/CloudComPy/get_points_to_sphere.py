import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2



point_cloud = o3d.io.read_point_cloud(r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_3.ply")

points = np.asarray(point_cloud.points)
color = np.asarray(point_cloud.colors)

print(color)