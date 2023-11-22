import cv2
import numpy as np
import open3d as o3d
import pyk4a
from pyk4a import Config, PyK4A
from helpers import colorize
import time

def save_images_periodically(k4a, file_suffix):
    counter = 0
    while True:
        capture = k4a.get_capture()
        if np.any(capture.color) and np.any(capture.depth) and np.any(capture.ir) and np.any(capture.depth_point_cloud):

            # Ihre bestehende Logik für die Verarbeitung und Anzeige der Bilder hier

            # Speichern Sie die Bilder jede Sekunde mit einem hochgezählten Dateinamen
            color_file_name = f"dataset/original_images/color_images/color_image_{file_suffix}_{counter}.png"
            cv2.imwrite(color_file_name, capture.color[:, :, :3])
#########################################################################################################

            depth_image = capture.depth
            depth_colored = cv2.applyColorMap(
                cv2.convertScaleAbs(capture.depth, alpha=0.18),
                cv2.COLORMAP_JET
                )
            depth_file_name = f"dataset/original_images/depth_images/normal/depth_image_{file_suffix}_{counter}.png"
            cv2.imwrite(depth_file_name, depth_colored)

#########################################################################################################

            depth_colored_colorized = colorize(capture.depth, (300, 1200), cv2.COLORMAP_JET)
            depth_file_name = f"dataset/original_images/depth_images/colorized/depth_image_colorized_{file_suffix}_{counter}.png"
            cv2.imwrite(depth_file_name, depth_colored_colorized)

#########################################################################################################
            
            infrared_image = capture.ir
            depth_image = capture.depth
            normalized_depth = (depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))
            adjusted_ir_image = infrared_image * normalized_depth
            infrared_file_name = f"dataset/original_images/infrared_images/infrared_image_{file_suffix}_{counter}.png"
            cv2.imwrite(infrared_file_name, adjusted_ir_image)

#########################################################################################################
            
            points = capture.depth_point_cloud.reshape((-1, 3))
            colors = capture.transformed_color[..., (2, 1, 0)].reshape((-1, 3))

            # Erstellen Sie eine Open3D PointCloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

            # Save pointcloud
            file_path = fr"dataset\original_images\pointcloud\pointcloud_{file_suffix}_{counter}.ply"
            o3d.io.write_point_cloud(file_path, pcd)

#########################################################################################################

            counter += 1
            time.sleep(1)  # Warten Sie eine Sekunde

def main():
    file_suffix = "black_5"
    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_720P,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=False,
        )
    )
    k4a.start()

    k4a.whitebalance = 4500
    assert k4a.whitebalance == 4500
    k4a.whitebalance = 4510
    assert k4a.whitebalance == 4510

    # Starten Sie den Prozess zum periodischen Speichern der Bilder
    save_images_periodically(k4a, file_suffix)

    k4a.stop()

if __name__ == "__main__":
    main()
