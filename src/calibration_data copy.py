#import k4a_calibration as k4a_calib
import pyk4a
from pyk4a import Config, PyK4A
import numpy as np
import cv2

# Open the Azure Kinect DK camera
k4a = PyK4A(
    Config(
        color_resolution=pyk4a.ColorResolution.RES_720P,
        camera_fps=pyk4a.FPS.FPS_5,
        depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
        synchronized_images_only=True,
    )
)

k4a.start()

calibration = k4a.calibration
calibration_raw = k4a.calibration_raw

# Capture a frame
capture = k4a.get_capture()

xmin = 250
ymin = 250
zmin = 100
xmax = 17500
ymax = 17500
zmax = 12000

# Get color image and point cloud from the capture
color_image = capture.color
point_cloud = capture.transformed_depth_point_cloud.reshape((-1, 3))

# Assuming you have a ROI in the point cloud defined by a bounding box (min_xyz, max_xyz)
roi_min = np.array([xmin, ymin, zmin])
roi_max = np.array([xmax, ymax, zmax])

# Extract points within the ROI
roi_mask = np.all((point_cloud >= roi_min) & (point_cloud <= roi_max), axis=1)
roi_points = point_cloud[roi_mask]
print("roi_points[:, :3]: ", roi_points[:3, :3])
# Project the ROI points to 2D coordinates in the color image
roi_2d = calibration.convert_3d_to_2d(roi_points[:3, :3], pyk4a.calibration.CalibrationType.DEPTH, pyk4a.calibration.CalibrationType.COLOR)

# Extract the corresponding region from the color image
roi_image = color_image.copy()
roi_image[~roi_mask] = 0  # Set pixels outside ROI to black

# Display the original color image with the ROI highlighted
cv2.imshow("Original Image with ROI", color_image)
cv2.imshow("ROI Extracted Image", roi_image)
cv2.waitKey(0)

# Stop the camera
k4a.stop()
cv2.destroyAllWindows()


# # Get color image and point cloud from the capture
# color_image = capture.color
# point_cloud = capture.transformed_depth_point_cloud

# # Assuming you have a 3D point in the point cloud (x, y, z)
# point_3d = np.array([x, y, z])

# # Project the 3D point to 2D coordinates in the color image
# point_2d = calibration.convert_3d_to_2d(point_3d, pyk4a.calibration.CalibrationType.COLOR)

# # Print the 2D coordinates in the color image
# print("2D Coordinates in Color Image:", point_2d)

# # Stop the camera
# k4a.stop()

