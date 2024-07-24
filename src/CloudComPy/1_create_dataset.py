import cv2
import numpy as np
import open3d as o3d
import pyk4a
from pyk4a import Config, PyK4A
from helpers import colorize

def main():
    # Festgelegter String für das Kürzel im Dateinamen
    # Camera-position_backround-color_iteration-number
    # file_suffix = "cameraPos-200_white_0"
    file_suffix = "250"
    good = 1
    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_1536P,  # 4K Farbauflösung
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=True,
        )
    )
    k4a.start()
    pyk4a.depth_image_to_color_camera
    # getters and setters directly get and set on device
    k4a.whitebalance = 4500
    assert k4a.whitebalance == 4500
    k4a.whitebalance = 4510
    assert k4a.whitebalance == 4510

    # Warten Sie auf eine Erfassung
    while True:
        capture = k4a.get_capture()
        if np.any(capture.color) and np.any(capture.depth) and np.any(capture.ir):
            break

#########################################################################################################

    # Show color image
    color_image = capture.color[:, :, :3]
    # cv2.imshow("Color Image", color_image)
    # cv2.waitKey(0)

    height, width, _ = color_image.shape
    top_margin = int(height * 0.20)
    bottom_margin = int(height * 0.75)
    left_margin = int(width * 0.30)
    right_margin = int(width * 0.65)

    # cropped_color_image = color_image[top_margin:bottom_margin, left_margin:right_margin]
    # cv2.imshow("Color Image", cropped_color_image)
    # cv2.waitKey(0)

    # Save color image
    if good == 1:
        color_file_name = f"src/CloudComPy/dataset/quality_good/rgb_image/color_image_{file_suffix}.png"
    else:
        color_file_name = f"src/CloudComPy/dataset/quality_bad/rgb_image/color_image_{file_suffix}.png"
    cv2.imwrite(color_file_name, color_image)


#########################################################################################################

    # Show normal depth image
    depth_image = capture.depth
    depth_colored = cv2.applyColorMap(
                cv2.convertScaleAbs(capture.depth, alpha=0.18),
                cv2.COLORMAP_JET
            )
    # cv2.imshow("Depth Image", depth_colored)
    # cv2.waitKey(0)

    height, width, _ = depth_colored.shape
    top_margin = int(height * 0.24)
    bottom_margin = int(height * 0.76)
    left_margin = int(width * 0.24)
    right_margin = int(width * 0.76)
   
    # Save normal depth image
    depth_file_name = f"dataset/depth_images/normal/depth_image_{file_suffix}.png"
    cv2.imwrite(depth_file_name, depth_colored)

#########################################################################################################

    # Show colorized depth image
    depth_colored_colorized = colorize(depth_image, (300, 1200), cv2.COLORMAP_JET) #COLORMAP_JET COLORMAP_HSV
    # cv2.imshow("Depth Image", depth_colored_colorized)
    # cv2.waitKey(0)

    # Save colorized depth image
    depth_file_name = f"dataset/depth_images/colorized/depth_image_colorized_{file_suffix}.png"
    cv2.imwrite(depth_file_name, depth_colored_colorized)


#########################################################################################################

    # Show infrared image
    infrared_image = capture.ir
    depth_image = capture.depth
    normalized_depth = (depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))
    adjusted_ir_image = infrared_image * normalized_depth
    # cv2.imshow("Infrared Image", adjusted_ir_image)
    # cv2.waitKey(0)

    # Save infrared image
    infrared_file_name = f"dataset/infrared_images/infrared_image_{file_suffix}.png"
    cv2.imwrite(infrared_file_name, adjusted_ir_image)

#########################################################################################################


    points = capture.depth_point_cloud.reshape((-1, 3))
    colors = capture.transformed_color[..., (2, 1, 0)].reshape((-1, 3))

    # Erstellen Sie eine Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

    # Save pointcloud
    if good == 1:
        file_path = f"src\CloudComPy\dataset\quality_good\og_pointcloud/pointcloud_{file_suffix}.ply"
    else: 
        file_path = f"src\CloudComPy\dataset\quality_bad\og_pointcloud/pointcloud_{file_suffix}.ply"
    o3d.io.write_point_cloud(file_path, pcd)

#########################################################################################################

    def remove_distant_points(point_cloud, max_distance, good, file_suffix):
        # Konvertiere die Pointcloud in ein Numpy-Array
        points = np.asarray(point_cloud.points)

        # Berechne die Entfernung jedes Punktes von der Kamera
        distances = np.linalg.norm(points, axis=1)

        # Finde Indizes der Punkte, die innerhalb der maximalen Entfernung liegen
        valid_indices = np.where(distances <= max_distance)[0]

        # Filtere die Punkte entsprechend den gefundenen Indizes
        filtered_points = points[valid_indices]
        filtered_colors = np.asarray(point_cloud.colors)[valid_indices]

        # Erstelle eine neue Pointcloud mit den gefilterten Punkten
        filtered_point_cloud = o3d.geometry.PointCloud()
        filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
        filtered_point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)

        if good == 1:
            file_path = f"src\CloudComPy\dataset\quality_good\cropped_pointcloud/pointcloud_copped_{file_suffix}.ply"
        else: 
            file_path = f"src\CloudComPy\dataset\quality_bad\cropped_pointcloud/pointcloud_copped_{file_suffix}.ply"
        
        o3d.io.write_point_cloud(file_path, filtered_point_cloud)

        return filtered_point_cloud

    filtered_pointcloud = remove_distant_points(pcd, 700, good, file_suffix)
#########################################################################################################
    k4a.stop()

if __name__ == "__main__":
    main()

