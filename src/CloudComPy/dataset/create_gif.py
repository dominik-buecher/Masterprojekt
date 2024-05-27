import open3d as o3d
import numpy as np
import imageio

# Lade deine Point Cloud Datei
path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\pointcloud.ply"
point_cloud = o3d.io.read_point_cloud(path)  # Ersetze "deine_pointcloud.ply" mit deinem Dateinamen

# Initiale Rotation, um die Point Cloud korrekt auszurichten
# Hier wird die Point Cloud um 180 Grad um die X-Achse gedreht, damit "oben" auch nach oben zeigt
initial_rotation = point_cloud.get_rotation_matrix_from_xyz((np.pi, 0, 0))
point_cloud.rotate(initial_rotation, center=(0, 0, 0))

# Erstelle eine Visualisierungsinstanz
vis = o3d.visualization.Visualizer()
vis.create_window(visible=False)
vis.add_geometry(point_cloud)

# Setze Kamera-Parameter
ctr = vis.get_view_control()
parameters = ctr.convert_to_pinhole_camera_parameters()

# Funktion um Bild aufzunehmen
def capture_view(vis):
    img = vis.capture_screen_float_buffer(False)
    return (np.asarray(img) * 255).astype(np.uint8)

# Anzahl der Frames und Rotationswinkel
num_frames = 72  # Erhöht die Anzahl der Frames für eine langsamere Rotation
rotation_angle = 360 / num_frames

images = []
for i in range(num_frames):
    # Rotieren um den Mittelpunkt der Point Cloud
    R = point_cloud.get_rotation_matrix_from_xyz((0, np.deg2rad(rotation_angle), 0))
    point_cloud.rotate(R, center=(0, 0, 0))
    vis.update_geometry(point_cloud)
    vis.poll_events()
    vis.update_renderer()
    img = capture_view(vis)
    images.append(img)

vis.destroy_window()


# Speichere die Bilder als GIF
imageio.mimsave(r'C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\pointcloud_rotation.gif', images, fps=10)  # fps = frames per second
