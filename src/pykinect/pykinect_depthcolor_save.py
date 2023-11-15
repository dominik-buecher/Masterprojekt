import cv2
import os
import pykinect_azure as pykinect
import numpy as np

if __name__ == "__main__":
    pykinect.initialize_libraries()

    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

    device = pykinect.start_device(config=device_config)

    cv2.namedWindow('Depth Image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Color Image', cv2.WINDOW_NORMAL)

    save_dir_color = "dataset/color_images"
    save_dir_depth = "dataset/depth_images"
    max_images = 10
    count_color = 0
    count_depth = 0

    if not os.path.exists(save_dir_color):
        os.mkdir(save_dir_color)
    if not os.path.exists(save_dir_depth):
        os.mkdir(save_dir_depth)

    while True:
        capture = device.update()

        ret_depth, depth_image = capture.get_colored_depth_image()
        ret_color, color_image = capture.get_color_image()

        if ret_depth and count_depth < max_images:
            count_depth += 1
            depth_filename = os.path.join(save_dir_depth, f"depth_{count_depth}.png")
            cv2.imwrite(depth_filename, depth_image)

        if ret_color and count_color < max_images:
            count_color += 1
            color_filename = os.path.join(save_dir_color, f"color_{count_color}.png")
            cv2.imwrite(color_filename, color_image)

        if count_color >= max_images and count_depth >= max_images:
            break

        if depth_image is not None:
            cv2.imshow('Depth Image', depth_image)

        if color_image is not None:
            cv2.imshow('Color Image', color_image)

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    #device.stop()
    device.close()

