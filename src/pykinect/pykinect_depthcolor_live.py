import cv2
import pykinect_azure as pykinect
import numpy as np

if __name__ == "__main__":
    # Initialize the library, if the library is not found, add the library path as an argument
    pykinect.initialize_libraries()

    # Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED

    # Start device
    device = pykinect.start_device(config=device_config)

    cv2.namedWindow('Depth Image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Color Image', cv2.WINDOW_NORMAL)

    while True:
        # Get capture
        capture = device.update()

        # Get the depth and color images from the capture
        #ret_depth, depth_image = capture.get_depth_image()
        ret_color, color_image = capture.get_color_image()
        ret_depth, depth_image = capture.get_colored_depth_image()
		
    
        
        if depth_image is None or color_image is None:
            continue

        cv2.imshow('Depth Image', depth_image)
        
        # Display the color image
        cv2.imshow('Color Image', color_image)

        # Press q key to stop
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    device.stop()
    device.close()

