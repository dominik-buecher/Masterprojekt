import cv2
import open3d as o3d
import pykinect_azure as pykinect

if __name__ == "__main__":
    # Initialisiere die Bibliothek
    pykinect.initialize_libraries()

    # Konfiguriere die Kamera
    device_config = pykinect.default_configuration
    device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED

    # Starte das Gerät
    device = pykinect.start_device(config=device_config)

    # Loop für die Live-Anzeige
    cv2.namedWindow('Transformed color', cv2.WINDOW_NORMAL)
    while True:
        # Erfasse die Daten
        capture = device.update()

        # Erhalte die transformierte Punktwolke
        ret_point, points = capture.get_transformed_pointcloud()

        # Erhalte das Farbbild im Tiefenkamera-Koordinatensystem
        ret_color, color_image = capture.get_color_image()

        if not ret_color or not ret_point:
            continue

        # Erstelle eine Open3D-Punktwolke
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(color_image / 255.0)

        # Zeige die Punktwolke mit Open3D an
        o3d.visualization.draw_geometries([pcd])

        # Zeige das Farbbild mit OpenCV an
        cv2.imshow('Transformed color', color_image)

        # Drücke die 'q'-Taste, um die Schleife zu beenden
        if cv2.waitKey(1) == ord('q'):
            break
