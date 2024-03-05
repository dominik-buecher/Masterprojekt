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
            color_resolution=pyk4a.ColorResolution.RES_720P,
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
        if np.any(capture.color):
            break


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


    # Speichern Sie das Tiefenbild
    # file_name = f"dataset/depth_images/depth_image_{file_suffix}.png"
    # cv2.imwrite(file_name, depth_colored)

    k4a.stop()

if __name__ == "__main__":
    main()