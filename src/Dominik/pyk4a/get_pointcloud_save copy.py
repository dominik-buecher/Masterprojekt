import numpy as np
import open3d as o3d
import pyk4a
from pyk4a import Config, PyK4A

def main():
    # Festgelegter String für das Kürzel im Dateinamen
    # Camera-position_backround-color_iteration-number
    file_suffix = "cameraPos-0_white_1"

    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_720P,
            camera_fps=pyk4a.FPS.FPS_5,
            depth_mode=pyk4a.DepthMode.WFOV_2X2BINNED,
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

    points1 = capture.depth_point_cloud.reshape((-1, 3))
    colors1 = capture.transformed_color[..., (2, 1, 0)].reshape((-1, 3))
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(points1)
    pcd1.colors = o3d.utility.Vector3dVector(colors1 / 255.0)

    file_path1 = fr"dataset\pointcloud_og.ply"
    o3d.io.write_point_cloud(file_path1, pcd1)

    # Begrenzen Sie den zentralen Bereich auf etwa 20%
    center_fraction = 0.2
    center_height = int(capture.depth.shape[0] * center_fraction)
    center_width = int(capture.depth.shape[1] * center_fraction)

    print("center_height", center_height)
    print("center_width", center_width)

    top_margin = int((capture.depth.shape[0] - center_height) / 2)
    bottom_margin = top_margin + center_height
    left_margin = int((capture.depth.shape[1] - center_width) / 2)
    right_margin = left_margin + center_width

    # Überprüfen, ob der zentrale Bereich gültige Punkte enthält
    if top_margin >= bottom_margin or left_margin >= right_margin:
        print("Der zentrale Bereich enthält keine gültigen Punkte.")
        return

    print("top_margin", top_margin)
    print("bottom_margin", bottom_margin)
    print("left_margin", left_margin)
    print("right_margin", right_margin)

    points2 = points1[top_margin:bottom_margin, left_margin:right_margin]
    colors2 = colors1[top_margin:bottom_margin, left_margin:right_margin]
    print(f"Number of points in the PointCloud: {len(pcd1.points)}")

    # Erstellen Sie eine Open3D PointCloud
    
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(points2.reshape((-1, 3)))
    pcd2.colors = o3d.utility.Vector3dVector(colors2.reshape((-1, 3)) / 255.0)
    print(f"Number of points in the PointCloud: {len(pcd2.points)}")

    o3d.visualization.draw_geometries([pcd2])

    file_path2 = fr"dataset\pointcloud_cropped.ply"
    o3d.io.write_point_cloud(file_path2, pcd2)

    k4a.stop()

if __name__ == "__main__":
    main()

