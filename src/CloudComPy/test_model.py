import open3d as o3d
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model

def read_point_cloud(ply_file):
    pcd = o3d.io.read_point_cloud(ply_file)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    return np.hstack((points, colors))

def load_and_prepare_point_cloud(ply_file, max_points):
    point_cloud = read_point_cloud(ply_file)
    padded_pc = np.pad(point_cloud, ((0, max_points - point_cloud.shape[0]), (0, 0)), mode='constant')
    return np.expand_dims(padded_pc, axis=0)

def main():
    model_path = r'C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\models\pointnet_model_v4.h5'
    test_ply_file = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes\pointcloud_copped_1.ply\mesh_cloud_color_green\mesh_cloud_1.ply"
    test_ply_file = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\meshes\pointcloud_copped_1.ply\mesh_cloud_color\mesh_cloud_10.ply"

    # Modell laden
    model = load_model(model_path)

    # Daten vorbereiten
    max_points = 1440  # Setze die maximale Anzahl der Punkte, wie im Trainingsskript
    test_data = load_and_prepare_point_cloud(test_ply_file, max_points)

    # Vorhersage
    prediction = model.predict(test_data)
    print('Prediction:', prediction)
    label = 'good' if prediction[0] > 0.5 else 'bad'
    print('Predicted label:', label)

if __name__ == "__main__":
    main()
