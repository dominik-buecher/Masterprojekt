import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

def point_cloud_to_depth_image(point_cloud, image_width, image_height, max_depth=800):
    # Erstelle ein leeres Tiefenbild
    depth_image = np.ones((image_height, image_width)) * max_depth

    # Iteriere über die Punkte der Punktwolke
    for point in point_cloud.points:
        # Konvertiere die 3D-Punkte in Pixelkoordinaten
        x = int((point[0] / max_depth + 0.5) * image_width)
        y = int((-point[1] / max_depth + 0.5) * image_height)

        # Überprüfe, ob der Punkt innerhalb der Bildgrenzen liegt
        if 0 <= x < image_width and 0 <= y < image_height:
            # Aktualisiere den Tiefenwert des entsprechenden Pixels
            depth_image[y, x] = min(depth_image[y, x], point[2])

    return depth_image

# Beispiel für die Verwendung des Skripts
point_cloud = o3d.io.read_point_cloud(r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\dataset\pointcloud\pointcloud_noBackround4.ply")  # Lade die Punktwolke
image_width = 640  # Breite des Tiefenbildes
image_height = 576  # Höhe des Tiefenbildes

# Konvertiere die Punktwolke in ein Tiefenbild
depth_image = point_cloud_to_depth_image(point_cloud, image_width, image_height)

# Zeige das Tiefenbild an (optional)

plt.imshow(depth_image, cmap='gray')
plt.show()
