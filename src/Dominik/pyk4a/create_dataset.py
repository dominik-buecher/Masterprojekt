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

    height, width, _ = color_image.shape
    top_margin = int(height * 0.20)
    bottom_margin = int(height * 0.75)
    left_margin = int(width * 0.30)
    right_margin = int(width * 0.65)

    cropped_color_image = color_image[top_margin:bottom_margin, left_margin:right_margin]
    cv2.imshow("Color Image", cropped_color_image)
    cv2.waitKey(0)
    # Save color image
    color_file_name = f"dataset/original_images/color_images/color_image_{file_suffix}.png"
    cv2.imwrite(color_file_name, color_image)
    color_file_name = f"dataset/cropped_images/color_images/color_image_cropped_{file_suffix}.png"
    cv2.imwrite(color_file_name, cropped_color_image)

#########################################################################################################

    # Show normal depth image
    depth_image = capture.depth
    depth_colored = cv2.applyColorMap(
                cv2.convertScaleAbs(capture.depth, alpha=0.18),
                cv2.COLORMAP_JET
            )
    cv2.imshow("Depth Image", depth_colored)
    cv2.waitKey(0)

    height, width, _ = depth_colored.shape
    top_margin = int(height * 0.24)
    bottom_margin = int(height * 0.76)
    left_margin = int(width * 0.24)
    right_margin = int(width * 0.76)
    
    cropped_depth_colored = depth_colored[top_margin:bottom_margin, left_margin:right_margin]
    cv2.imshow("Depth Image", cropped_depth_colored)
    cv2.waitKey(0)
   
    # Save normal depth image
    depth_file_name = f"dataset/original_images/depth_images/normal/depth_image_{file_suffix}.png"
    cv2.imwrite(depth_file_name, depth_colored)
    depth_file_name = f"dataset/cropped_images/depth_images/normal/depth_image_cropped_{file_suffix}.png"
    cv2.imwrite(depth_file_name, cropped_depth_colored)

#########################################################################################################

    # Show colorized depth image
    depth_colored_colorized = colorize(depth_image, (None, 5000), cv2.COLORMAP_HSV)
    cv2.imshow("Depth Image", depth_colored_colorized)
    cv2.waitKey(0)

    height, width, _ = depth_colored_colorized.shape
    top_margin = int(height * 0.24)
    bottom_margin = int(height * 0.76)
    left_margin = int(width * 0.24)
    right_margin = int(width * 0.76)
    
    cropped_depth_colored = depth_colored_colorized[top_margin:bottom_margin, left_margin:right_margin]
    cv2.imshow("Depth Image", cropped_depth_colored)
    cv2.waitKey(0)
    # Save colorized depth image
    depth_file_name = f"dataset/original_images/depth_images/colorized/depth_image_colorized_{file_suffix}.png"
    cv2.imwrite(depth_file_name, depth_colored_colorized)
    depth_file_name = f"dataset/cropped_images/depth_images/colorized/depth_image_colorized_cropped_{file_suffix}.png"
    cv2.imwrite(depth_file_name, cropped_depth_colored)

#########################################################################################################

    # Show infrared image
    infrared_image = capture.ir
    depth_image = capture.depth
    normalized_depth = (depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))
    adjusted_ir_image = infrared_image * normalized_depth
    cv2.imshow("Infrared Image", adjusted_ir_image)
    cv2.waitKey(0)

    height, width = adjusted_ir_image.shape
    top_margin = int(height * 0.24)
    bottom_margin = int(height * 0.76)
    left_margin = int(width * 0.24)
    right_margin = int(width * 0.76)
    
    cropped_ir_image = adjusted_ir_image[top_margin:bottom_margin, left_margin:right_margin]
    cv2.imshow("Depth Image", cropped_ir_image)
    cv2.waitKey(0)

    # Save infrared image
    infrared_file_name = f"dataset/original_images/infrared_images/infrared_image_{file_suffix}.png"
    cv2.imwrite(infrared_file_name, adjusted_ir_image)
    infrared_file_name = f"dataset/cropped_images/infrared_images/infrared_image_cropped_{file_suffix}.png"
    cv2.imwrite(infrared_file_name, cropped_ir_image)

#########################################################################################################


    points = capture.depth_point_cloud.reshape((-1, 3))
    colors = capture.transformed_color[..., (2, 1, 0)].reshape((-1, 3))

    # Erstellen Sie eine Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

    # Save pointcloud
    file_path = fr"dataset\original_images\pointcloud\pointcloud_{file_suffix}.ply"
    o3d.io.write_point_cloud(file_path, pcd)

#########################################################################################################
    k4a.stop()

if __name__ == "__main__":
    main()

