import cv2
import numpy as np
import open3d as o3d
import pyk4a
from pyk4a import Config, PyK4A
from helpers import colorize

def main():
    # Festgelegter String für das Kürzel im Dateinamen
    # Camera-position_backround-color_iteration-number
    file_suffix = "cameraPos-0_white_2"

    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_720P,  # Farbauflösung
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=False,
        )
    )
    k4a.start()

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
    cv2.imshow("Color Image", color_image)
    cv2.waitKey(0)
    # Save color image
    color_file_name = f"dataset/color_images/color_image_{file_suffix}.png"
    cv2.imwrite(color_file_name, color_image)

#########################################################################################################

    # Show normal depth image
    depth_image = capture.depth
    depth_colored = cv2.applyColorMap(
                cv2.convertScaleAbs(capture.depth, alpha=0.18),
                cv2.COLORMAP_JET
            )
    cv2.imshow("Depth Image", depth_colored)
    cv2.waitKey(0)
    # Save normal depth image
    depth_file_name = f"dataset/depth_images/normal/depth_image_{file_suffix}.png"
    cv2.imwrite(depth_file_name, depth_colored)

#########################################################################################################

    # Show colorized depth image
    depth_colored_colorized = colorize(depth_image, (None, 5000), cv2.COLORMAP_HSV)
    cv2.imshow("Depth Image", depth_colored_colorized)
    cv2.waitKey(0)
    # Save colorized depth image
    depth_file_name = f"dataset/depth_images/colorized/depth_image_colorized_{file_suffix}.png"
    cv2.imwrite(depth_file_name, depth_colored_colorized)

#########################################################################################################

    # Show infrared image
    infrared_image = capture.ir
    depth_image = capture.depth
    normalized_depth = (depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))
    adjusted_ir_image = infrared_image * normalized_depth
    cv2.imshow("Infrared Image", adjusted_ir_image)
    cv2.waitKey(0)
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
    file_path = fr"dataset\pointcloud\pointcloud_{file_suffix}.ply"
    o3d.io.write_point_cloud(file_path, pcd)

#########################################################################################################
    k4a.stop()

if __name__ == "__main__":
    main()

