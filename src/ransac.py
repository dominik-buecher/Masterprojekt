import open3d as o3d
from sklearn.linear_model import RANSACRegressor
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from joblib import dump, load

def crop_pointcloud(pointcloud_file, max_distance=1.0):
    # Schritt 1: Einlesen der Pointcloud
    pointcloud = o3d.io.read_point_cloud(pointcloud_file)

    # Schritt 2: Zentriere und skaliere die Punktwolke
    #pointcloud.scale(1/1000, center=pointcloud.get_center())
    o3d.visualization.draw_geometries([pointcloud])
    # Schritt 3: Extrahiere Punkte und Farben direkt über die Open3D-API
    print("pointcloud.points: ", pointcloud.points)
    points = np.asarray(pointcloud.points)
    print("points: ", points)
    points[:, 1:] *= -1
    print("points: ", points)
    colors = np.asarray(pointcloud.colors)

    # Schritt 4: Berechne die Entfernungen der Punkte von der Kamera
    distances = np.linalg.norm(points, axis=1)

    # Schritt 5: Filtere Punkte, die weiter als 1 Meter entfernt sind
    mask = distances <= max_distance
    cropped_points = points[mask]
    cropped_colors = colors[mask]

    # Schritt 6: Erstelle eine neue Pointcloud mit den angepassten Koordinaten und Farben
    cropped_pointcloud = o3d.geometry.PointCloud()
    cropped_pointcloud.points = o3d.utility.Vector3dVector(cropped_points)
    cropped_pointcloud.colors = o3d.utility.Vector3dVector(cropped_colors)

    # Schritt 7: Zeige Informationen über die Punktwolken an
    print("Ursprüngliche Anzahl der Punkte:", len(points))
    print("Zugeschnittene Anzahl der Punkte:", len(cropped_points))
    print("Anzahl der entfernten Punkte:", len(points) - len(cropped_points))
    print("Entfernungen der ersten 10 Punkte:\n", distances[:10])
    print("Beispielkoordinaten vor Zuschneiden:\n", points[:5, :])
    print("Beispielkoordinaten nach Zuschneiden:\n", cropped_points[:5, :])

    # Schritt 8: Visualisiere die zugeschnittene Pointcloud
    o3d.visualization.draw_geometries([cropped_pointcloud])






def crop_pointcloud5(pointcloud_file, max_distance=2.0):
    # Schritt 1: Einlesen der Pointcloud
    pointcloud = o3d.io.read_point_cloud(pointcloud_file)

    # Schritt 2: Visualisiere die Originalpunktwolke
    o3d.visualization.draw_geometries([pointcloud])

    # Schritt 3: Extrahiere Punkte und Farben direkt über die Open3D-API
    points = np.asarray(pointcloud.points)
    colors = np.asarray(pointcloud.colors)

    # Schritt 4: Berechne die Entfernungen der Punkte von der Ursprungsposition
    distances = np.linalg.norm(points, axis=1)

    # Schritt 5: Filtere Punkte, die weiter als 1 Meter entfernt sind
    mask = distances <= max_distance
    cropped_points = points[mask]
    cropped_colors = colors[mask]

    # Schritt 6: Erstelle eine neue Pointcloud mit den angepassten Koordinaten und Farben
    cropped_pointcloud = o3d.geometry.PointCloud()
    cropped_pointcloud.points = o3d.utility.Vector3dVector(cropped_points)
    cropped_pointcloud.colors = o3d.utility.Vector3dVector(cropped_colors)

    print("Anzahl der Punkte vor Zuschneiden:", len(points))
    print("Anzahl der Punkte nach Zuschneiden:", len(cropped_points))
    print("Anzahl der Punkte nach Zuschneiden:", len(cropped_colors))
    # Schritt 7: Visualisiere die zugeschnittene Pointcloud
    o3d.visualization.draw_geometries([cropped_pointcloud])

def crop_pointcloud4(pointcloud_file, max_distance=1.0):
    # Schritt 1: Einlesen der Pointcloud
    pointcloud = o3d.io.read_point_cloud(pointcloud_file)
    print("Originalpunktwolke (nach Einlesen): ", np.asarray(pointcloud.points))
    print("Originalfarben (nach Einlesen): ", np.asarray(pointcloud.colors))

    o3d.visualization.draw_geometries([pointcloud])
    points = np.asarray(pointcloud.points)
    colors = np.asarray(pointcloud.colors)
    print("Originalpunktwolke: ", points)

    distances = np.linalg.norm(points, axis=1)
    print("Entfernungen: ", distances)

    mask = distances <= max_distance
    cropped_points = points[mask]
    cropped_colors = colors[mask]
    print("Maske: ", mask)
    print("Zugeschnittene Punkte: ", cropped_points)

    if len(cropped_points) == 0:
        print("Es gibt keine sichtbaren Punkte nach dem Zuschneiden.")
        return

    cropped_pointcloud = o3d.geometry.PointCloud()
    cropped_pointcloud.points = o3d.utility.Vector3dVector(cropped_points)
    cropped_pointcloud.colors = o3d.utility.Vector3dVector(cropped_colors)


    o3d.visualization.draw_geometries([cropped_pointcloud])







def extract_grapes(pointcloud_file):

    pointcloud = o3d.io.read_point_cloud(pointcloud_file)
    
    points = np.asarray(pointcloud.points)
    colors = np.asarray(pointcloud.colors)

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
    epsilon = 1000  # => Schwelle für die Einbeziehung von Punkten als Inliers

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
    #pointcloud_file = "data/pointcloud_white_2_16.ply"
    pointcloud_file = "data/pointcloud_white_3_75.ply"
    extract_grapes(pointcloud_file)
    # crop_pointcloud(pointcloud_file)

