import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

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


rotation_matrix = np.array([[r11, r12, r13],
                            [r21, r22, r23],
                            [r31, r32, r33]])

#Translationsvektor
tx = -0.032028596848249435 * 1000
ty = -0.0020339391194283962 * 1000
tz = 0.0038644461892545223 * 1000


translation_vector = np.array([tx, ty, tz])

# Funktion zur Projektion der Pointcloud auf das RGB-Bild
def project_point_cloud_to_image(point_cloud, intrinsic_matrix, rotation_matrix, translation_vector, image_width, image_height):
    # Konvertiere die Pointcloud in ein Numpy-Array
    points = np.asarray(point_cloud.points)

    # Transformiere die Punkte in das Kamerakoordinatensystem
    transformed_points = np.dot(rotation_matrix, points.T).T + translation_vector

    # Projiziere die 3D-Punkte auf die 2D-Bildebene
    projected_points = np.dot(intrinsic_matrix, transformed_points.T).T
    projected_points[:, 0] /= projected_points[:, 2]
    projected_points[:, 1] /= projected_points[:, 2]

    # Runde die projizierten Pixelkoordinaten auf ganze Zahlen
    pixel_coordinates = np.round(projected_points[:, :2]).astype(int)

    return pixel_coordinates

# Lade die Pointcloud und das RGB-Bild
point_cloud = o3d.io.read_point_cloud(r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_3.ply")
rgb_image = plt.imread(r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\dataset\color_images\color_image_noBackround5.png")




cx = 0.50077033042907715
cy = 0.50537955760955811
fx = 0.47449326515197754 
fy = 0.63248747587203979  



# Intrinsische Kameramatrix
model_parameters_rgb = [0.50077033042907715, 0.50537955760955811, 0.47449326515197754, 0.63248747587203979, 0.27764144539833069, -2.4767875671386719, 1.5355490446090698, 0.15892954170703888, -2.2932455539703369, 1.4548124074935913, 0, 0, -7.528195419581607E-5, 0.00012816225353162736]

# Sensor size
sensor_width_rgb = 4096
sensor_height_rgb = 3072

## image size
image_width_rgb = 2048
image_height_rgb = 1536



# Scale the intrisic parameters with the image size
# c
model_parameters_rgb[0] *= image_width_rgb 
model_parameters_rgb[1] *= image_height_rgb

# f
model_parameters_rgb[2] *=  image_width_rgb
# is used to skale the image because the image hight cloud be smaller than the sensor size
model_parameters_rgb[3] *=  image_width_rgb * (sensor_height_rgb/sensor_width_rgb)

# Intrinsische Matrix erstellen
intrinsic_matrix = np.array([[model_parameters_rgb[2], 0, model_parameters_rgb[0]],
                              [0, model_parameters_rgb[3], model_parameters_rgb[1]],
                              [0, 0, 1]])




# Projektion der Pointcloud auf das RGB-Bild
pixel_coordinates = project_point_cloud_to_image(point_cloud, intrinsic_matrix, rotation_matrix, translation_vector, rgb_image.shape[1], rgb_image.shape[0])

# Färben des RGB-Bilds basierend auf den projizierten Pixelkoordinaten
for x, y in pixel_coordinates:
    if 0 <= x < rgb_image.shape[1] and 0 <= y < rgb_image.shape[0]:
        rgb_image[y, x] = [255, 0, 0]  # Markierung des Pixels mit Rot, um die Punkte der Pointcloud zu kennzeichnen

# Anzeigen des RGB-Bilds mit markierten Punkten
plt.imshow(rgb_image)
plt.show()

