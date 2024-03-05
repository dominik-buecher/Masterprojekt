import numpy as np
import open3d as o3d
import pyk4a
from pyk4a import Config, PyK4A

def main():
    # Festgelegter String für das Kürzel im Dateinamen
    # Camera-position_backround-color_iteration-number
    file_suffix = "kugel"#"cameraPos-0_white_1"

    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_720P,
            camera_fps=pyk4a.FPS.FPS_5,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=True,
        )
    )
    k4a.start()

    # getters and setters directly get and set on device
    k4a.whitebalance = 4500
    assert k4a.whitebalance == 4500
    k4a.whitebalance = 4510
    assert k4a.whitebalance == 4510

    while True:
        capture = k4a.get_capture()
        if np.any(capture.depth) and np.any(capture.color):
            break

    while True:
        capture = k4a.get_capture()
        if np.any(capture.depth) and np.any(capture.color):
            break

    points = capture.depth_point_cloud.reshape((-1, 3))
    colors = capture.transformed_color[..., (2, 1, 0)].reshape((-1, 3))

    # Erstellen Sie eine Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

    # Speichern Sie die PointCloud im PLY-Format
    file_path = f"data/pointcloud_{file_suffix}.ply"
    o3d.io.write_point_cloud(file_path, pcd)

    k4a.stop()

if __name__ == "__main__":
    main()
