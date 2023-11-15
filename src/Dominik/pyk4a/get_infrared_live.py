import cv2
import numpy as np
import pyk4a
from pyk4a import Config, PyK4A

def main():
    # Festgelegter String für das Kürzel im Dateinamen
    # Camera-position_backround-color_iteration-number
    file_suffix = "cameraPos-0_white_1"

    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.OFF,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=False,
            #camera_fps=pyk4a.FPS.FPS_30,  # Anpassung der Bildrate nach Bedarf
            #depth_mode=pyk4a.DepthMode.PASSIVE_IR  # Infrarot-Modus
        )
    )
    k4a.start()

    # Warten Sie auf eine Erfassung
    while True:
        capture = k4a.get_capture()
        if np.any(capture.ir) and np.any(capture.depth):
            ir_image = capture.ir
            depth_image = capture.depth

            # Hier können Sie die Depth-Informationen verwenden, um die Infrarotbilder anzupassen
            normalized_depth = (depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))
            adjusted_ir_image = ir_image * normalized_depth
            file_name = f"dataset/infrared_images/ir_image_{file_suffix}.png"
            cv2.imwrite(file_name, adjusted_ir_image)
            cv2.imshow("Infrared Image", adjusted_ir_image)

           
            

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    k4a.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()