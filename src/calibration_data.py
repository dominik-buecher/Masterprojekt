import k4a_calibration as k4a_calib
import pyk4a
from pyk4a import Config, PyK4A
import numpy as np

# Open the Azure Kinect DK camera
#k4a = k4a_calib.Device()

k4a = PyK4A(
    Config(
        color_resolution=pyk4a.ColorResolution.RES_720P,
        camera_fps=pyk4a.FPS.FPS_5,
        depth_mode=pyk4a.DepthMode.WFOV_2X2BINNED,
        synchronized_images_only=True,
    )
)

k4a.start()

calibration = k4a.calibration
calibration_raw = k4a.calibration_raw

print("calibration: ",calibration)
print("calibration_raw: ", calibration_raw)

# calibration = k4a.load_calibration_json()
# calibration = k4a.save_calibration_json_calibration_json()

# Get calibration data
calibration = k4a.get_calibration()

# Extract intrinsic parameters of the RGB camera
rgb_camera_calibration = calibration.color_camera_calibration
fx, fy = rgb_camera_calibration.intrinsics.parameters.param.fx, rgb_camera_calibration.intrinsics.parameters.param.fy
cx, cy = rgb_camera_calibration.intrinsics.parameters.param.cx, rgb_camera_calibration.intrinsics.parameters.param.cy

# Extract intrinsic parameters of the depth camera
depth_camera_calibration = calibration.depth_camera_calibration
fx_depth, fy_depth = depth_camera_calibration.intrinsics.parameters.param.fx, depth_camera_calibration.intrinsics.parameters.param.fy
cx_depth, cy_depth = depth_camera_calibration.intrinsics.parameters.param.cx, depth_camera_calibration.intrinsics.parameters.param.cy

# Extract extrinsic parameters
extrinsics = calibration.extrinsics.rotation

# Close the Azure Kinect DK camera
k4a.close()

# Use the extracted parameters for further processing


# import open3d as o3d
# import cv2

# Assumption: Load calibration information with the Azure Kinect SDK
# (The exact implementation depends on the SDK version)

# Intrinsic parameters for the RGB camera
intrinsics_rgb = calibration.color_camera_calibration.intrinsics

# Intrinsic parameters for the depth camera
intrinsics_depth = calibration.depth_camera_calibration.intrinsics

# Extrinsic parameters (rotation matrix and translation vector)
extrinsics = calibration.extrinsics


# Transformation from 3D world coordinates to 2D pixels on the RGB image
transformation_rgb = intrinsics_rgb.extrinsic.dot(extrinsics.rotation)
transformation_rgb[:3, 3] += extrinsics.translation

# # Assumption: Load the point cloud and RGB image
# pointcloud = o3d.io.read_point_cloud("path/to/pointcloud.ply")
# rgb_image = cv2.imread("path/to/rgb_image.jpg")

# min_x = 
# min_y = 
# min_z = 

# max_x = 
# max_y =
# max_z = 

# roi_min = [min_x, min_y, min_z]
# roi_max = [max_x, max_y, max_z]

# # Perform the transformation
# transformed_pointcloud = o3d.geometry.PointCloud()
# transformed_pointcloud.points = o3d.utility.Vector3dVector(
#     np.dot(np.asarray(pointcloud.points), transformation_rgb.T)
# )

# roi_image = rgb_image[y_min:y_max, x_min:x_max]

# cv2.imshow(roi_image)

# # Crop the ROI from the RGB image based on the transformed point cloud
# # (The exact implementation depends on your ROI definition)

# # Display the transformed point cloud and RGB image (for testing)
# o3d.visualization.draw_geometries([transformed_pointcloud, rgb_image])
