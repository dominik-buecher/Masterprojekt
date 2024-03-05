import cv2
import numpy as np
import open3d as o3d
import pyk4a
from pyk4a import Config, PyK4A
from helpers import colorize

def main():
    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_720P,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=True,
        )
    )
    k4a.start()

    while True:
        capture = k4a.get_capture()

        if np.any(capture.color):
            # Zeige das Farbbild an
            cv2.imshow("Color", capture.color[:, :, :3])

        if np.any(capture.depth):
            # Zeige das Tiefenbild an
            # depth_colored = cv2.applyColorMap(
                # cv2.convertScaleAbs(capture.depth, alpha=0.03),
                # cv2.COLORMAP_JET
            #)
            depth_colored = colorize(capture.depth, (None, 5000), cv2.COLORMAP_HSV)
            
            cv2.imshow("Depth", depth_colored)

        key = cv2.waitKey(10)
        if key != -1:
            cv2.destroyAllWindows()
            break

    k4a.stop()

if __name__ == "__main__":
    main()
