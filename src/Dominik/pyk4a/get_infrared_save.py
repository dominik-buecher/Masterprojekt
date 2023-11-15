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
            color_resolution=pyk4a.ColorResolution.OFF,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=False,
            camera_fps=pyk4a.FPS.FPS_30,  # Anpassung der Bildrate nach Bedarf
            #ir_mode=pyk4a.IRMode.NFOV_UNBINNED  # Infrarot-Modus
        )
    )
    k4a.start()

    # getters and setters directly get and set on device
    # k4a.whitebalance = 4500
    # assert k4a.whitebalance == 4500
    # k4a.whitebalance = 4510
    # assert k4a.whitebalance == 4510

    # Warten Sie auf eine Erfassung
    while True:
        capture = k4a.get_capture()
        if np.any(capture.ir):
            break

    # Verarbeiten und anzeigen Sie das Infrarotbild
    ir_image = capture.ir
    cv2.imshow("Infrared Image", ir_image)
    cv2.waitKey(0)

    # Speichern Sie das Infrarotbild
    file_name = f"dataset/infrared_images/ir_image_{file_suffix}.png"
    cv2.imwrite(file_name, ir_image)

    k4a.stop()

if __name__ == "__main__":
    main()
