# https://github.com/ibaiGorordo/pyKinectAzure/blob/master/examples/exampleDepthImage.py


import cv2
import pykinect_azure as pykinect
import numpy as np

def enhance_depth_color(depth_image):
    # Normalisieren Sie die Tiefendaten in den Bereich [0, 1]
    normalized_depth = (depth_image - 300) / 700  # 30 cm bis 1 Meter Bereich
    
    # Begrenzen Sie die Werte auf den gewünschten Bereich
    normalized_depth = np.clip(normalized_depth, 0, 1)
    
    # Erstellen Sie ein RGB-Bild, bei dem die Farbzuweisung von Blau bis Rot erfolgt
    depth_color = cv2.applyColorMap((normalized_depth * 255).astype(np.uint8), cv2.COLORMAP_JET)
    print("depth_color:  ", depth_color)
    # Alles, was weiter als 1 Meter entfernt ist, wird komplett rot
    red_mask = depth_image > 1000
    depth_color[red_mask] = (0, 0, 255)  # Alle Kanäle auf (0, 0, 255) setzen

    # Alles, was näher als 30 cm ist, wird komplett blau
    blue_mask = depth_image < 300
    depth_color[blue_mask] = (255, 0, 0)  # Alle Kanäle auf (255, 0, 0) setzen

    return depth_color

if __name__ == "__main__":
    pykinect.initialize_libraries()

    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

    device = pykinect.start_device(config=device_config)

    cv2.namedWindow('Depth Image', cv2.WINDOW_NORMAL)
    while True:
        # Get capture
        capture = device.update()

        # Get the color depth image from the capture
        ret, depth_image = capture.get_colored_depth_image()

        if not ret:
            continue

        # Verbessern Sie die Farbzuweisung der Tiefendaten
        depth_color = enhance_depth_color(depth_image)

        # Plot the enhanced depth color image
        cv2.imshow('Depth Image', depth_color)

        # Press q key to stop
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    device.stop()
    device.close()
