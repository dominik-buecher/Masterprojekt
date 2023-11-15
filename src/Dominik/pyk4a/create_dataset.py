import cv2
import numpy as np
import pyk4a
from pyk4a import Config, PyK4A
from helpers import colorize

def main():
    # Festgelegter String für das Kürzel im Dateinamen
    # Camera-position_backround-color_iteration-number
    file_suffix = "cameraPos-0_white_1"

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
        if np.any(capture.color) and np.any(capture.depth):
            break
    # Verarbeiten und anzeigen Sie das Farbbild
    color_image = capture.color[:, :, :3]
    cv2.imshow("Color Image", color_image)
    cv2.waitKey(0)

    # Speichern Sie das Farbbild
    color_file_name = f"dataset/color_images/color_image_{file_suffix}.png"
    cv2.imwrite(color_file_name, color_image)

    # Verarbeiten und anzeigen Sie das Tiefenbild
    depth_colored = colorize(capture.depth, (None, 5000), cv2.COLORMAP_HSV)
    cv2.imshow("Depth Image", depth_colored)
    cv2.waitKey(0)

    # Speichern Sie das Tiefenbild
    depth_file_name = f"dataset/depth_images/depth_image_{file_suffix}.png"
    cv2.imwrite(depth_file_name, depth_colored)

    k4a.stop()

if __name__ == "__main__":
    main()



