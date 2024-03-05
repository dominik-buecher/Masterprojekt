import numpy as np
import pyk4a
from pyk4a import Config, PyK4A
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def update_plot(points, colors, ax):
    ax.clear()
    ax.scatter(
        points[:, 0],
        points[:, 1],
        points[:, 2],
        s=1,
        c=colors / 255,
    )
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_xlim(-2000, 2000)
    ax.set_ylim(-2000, 2000)
    ax.set_zlim(0, 4000)
    ax.view_init(elev=-90, azim=-90)

def main():
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

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    while True:
        capture = k4a.get_capture()
        if np.any(capture.depth) and np.any(capture.color):
            points = capture.depth_point_cloud.reshape((-1, 3))
            colors = capture.transformed_color[..., (2, 1, 0)].reshape((-1, 3))

            # Update the plot with live data
            update_plot(points, colors, ax)
            plt.pause(0.01)  # Pause to allow the plot to update

    k4a.stop()

if __name__ == "__main__":
    main()
