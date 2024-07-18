import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2
# conda activate CloudComPy310
# cd C:\Users\domin\Documents\Studium\Master\CloudComPy310
# env
# python C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\test2.py

# Rotationsmatrix 
r11 = 0.99998950958251953
r12 = 0.004511446226388216
r13 = -0.0008156061521731317
r21 = -0.0044100326485931873
r22 = 0.99519461393356323
r23 = 0.09781724214553833
r31 = 0.0012529840460047126
r32 = -0.09781261533498764
r33 = 0.99520403146743774

# r11 = 1
# r12 = 0
# r13 = 0
# r21 = 0
# r22 = 1
# r23 = 0
# r31 = 0
# r32 = 0
# r33 = 1

rotation_matrix = np.array([[r11, r12, r13],
                            [r21, r22, r23],
                            [r31, r32, r33]])

#Translationsvektor
tx = -0.032028596848249435 * 1000
ty = -0.0020339391194283962 * 1000
tz = 0.0038644461892545223 * 1000

# tx = -0.032028596848249435
# ty = -0.0020339391194283962
# tz = 0.0038644461892545223

# tx = 0
# ty = 0
# tz = 0

translation_vector = np.array([tx, ty, tz])

# Funktion zur Projektion der Pointcloud auf das RGB-Bild
def project_point_cloud_to_image(point_cloud, intrinsic_matrix, rotation_matrix, translation_vector, image_width, image_height):
    # Konvertiere die Pointcloud in ein Numpy-Array
    points = np.asarray(point_cloud.points)
    #points = points * 1000
    # Transformiere die Punkte in das Kamerakoordinatensystem
    transformed_points = np.dot(rotation_matrix, points.T).T + translation_vector
    points_2d_rgb, _ = cv2.projectPoints(points, rotation_matrix, translation_vector, intrinsic_matrix, None)

    #transformed_points = np.dot(rotation_matrix.T, (points - translation_vector).T).T

    #transformed_points = np.dot(rotation_matrix, (points + translation_vector).T).T

    # Projiziere die 3D-Punkte auf die 2D-Bildebene
    projected_points = np.dot(intrinsic_matrix, transformed_points.T).T
    projected_points[:, 0] /= projected_points[:, 2]
    projected_points[:, 1] /= projected_points[:, 2]

    # Runde die projizierten Pixelkoordinaten auf ganze Zahlen
    pixel_coordinates = np.round(projected_points[:, :2]).astype(int)

    return points_2d_rgb

# Lade die Pointcloud und das RGB-Bild
#point_cloud = o3d.io.read_point_cloud(r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data6\mesh_24.ply")
point_cloud = o3d.io.read_point_cloud(r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data6\pointcloud_24.ply")
rgb_image = plt.imread(r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\dataset\color_images\color_image_noBackround3.png")




cx = 0.50077033042907715
cy = 0.50537955760955811
fx = 0.47449326515197754 
fy = 0.63248747587203979  



# # Intrinsische Kameramatrix
# intrinsic_matrix = np.array([[fx, 0, cx],
#                               [0, fy, cy],
#                               [0, 0, 1]])


# # Skalierungsfaktoren für die intrinsischen Parameter berechnen
# cx_scaled = cx * rgb_image.shape[1]
# cy_scaled = cy * rgb_image.shape[0]
# fx_scaled = fx * rgb_image.shape[1]
# fy_scaled = fy * rgb_image.shape[0]


model_parameters_rgb = [0.50077033042907715, 0.50537955760955811, 0.47449326515197754, 0.63248747587203979, 0.27764144539833069, -2.4767875671386719, 1.5355490446090698, 0.15892954170703888, -2.2932455539703369, 1.4548124074935913, 0, 0, -7.528195419581607E-5, 0.00012816225353162736]
sensor_width_rgb = 4096
sensor_height_rgb = 3072
image_width_rgb = 1280
image_height_rgb = 720

scale_x = image_width_rgb / sensor_width_rgb
scale_y = image_height_rgb / sensor_height_rgb


vergößerung1 = 1000
vergößerung2 = 1000
# Intrinsische Kameraparameter skalieren
# model_parameters_rgb[0] *= scale_x * vergößerung1
# model_parameters_rgb[1] *= scale_y * vergößerung2
# model_parameters_rgb[2] *= scale_x * vergößerung1
# model_parameters_rgb[3] *= scale_y * vergößerung2


# model_parameters_rgb[0] *= 1000/1.25
# model_parameters_rgb[1] *= 1000/1.25
# model_parameters_rgb[2] *= 1000/1.25
# model_parameters_rgb[3] *= 1000/1.25

# c
model_parameters_rgb[0] *= 1280
model_parameters_rgb[1] *= 720

# f
# model_parameters_rgb[2] *= 1/0.00125
# model_parameters_rgb[3] *= 1/0.00125

model_parameters_rgb[2] *= 1280
model_parameters_rgb[3] *= 720

# Intrinsische Matrix erstellen
intrinsic_matrix = np.array([[model_parameters_rgb[2], 0, model_parameters_rgb[0]],
                              [0, model_parameters_rgb[3], model_parameters_rgb[1]],
                              [0, 0, 1]])




model_parameters_depth = [0.50057387351989746,0.50801026821136475,0.49302750825881958,
                 0.49317404627799988,2.31860613822937,1.3959437608718872,
                 0.070345945656299591,2.6560931205749512,2.1336357593536377,
                 0.37754881381988525,0,0,-6.2750790675636381E-5,-6.8999062932562083E-5]  # Hier die Kalibrierungsparameter der Tiefenkamera einfügen
sensor_width_depth = 1024  # Hier die Sensorbreite der Tiefenkamera einfügen
sensor_height_depth = 1024  # Hier die Sensorhöhe der Tiefenkamera einfügen
scale_x_depth = image_width_rgb / sensor_width_depth
scale_y_depth = image_height_rgb / sensor_height_depth

model_parameters_depth[0] *= scale_x_depth * 1000
model_parameters_depth[1] *= scale_y_depth * 1000
model_parameters_depth[2] *= scale_x_depth * 1000
model_parameters_depth[3] *= scale_y_depth * 1000

# intrinsic_matrix = np.array([[model_parameters_depth[2], 0, model_parameters_depth[0]],
#                                    [0, model_parameters_depth[3], model_parameters_depth[1]],
#                                    [0, 0, 1]])




# # Intrinsische Kameramatrix mit skalierten Parametern erstellen
# intrinsic_matrix = np.array([[fx_scaled, 0, cx_scaled],
#                               [0, fy_scaled, cy_scaled],
#                               [0, 0, 1]])

# Projektion der Pointcloud auf das RGB-Bild
pixel_coordinates = project_point_cloud_to_image(point_cloud, intrinsic_matrix, rotation_matrix, translation_vector, rgb_image.shape[1], rgb_image.shape[0])

# Färben des RGB-Bilds basierend auf den projizierten Pixelkoordinaten
for x, y in pixel_coordinates:
    if 0 <= x < rgb_image.shape[1] and 0 <= y < rgb_image.shape[0]:
        rgb_image[y, x] = [255, 0, 0]  # Markierung des Pixels mit Rot, um die Punkte der Pointcloud zu kennzeichnen

# Anzeigen des RGB-Bilds mit markierten Punkten
plt.imshow(rgb_image)
plt.show()

