import k4a

def main():
    k4a.device_open()

    # Konfigurieren Sie die Kameraeinstellungen
    config = k4a.device_configuration_t()
    config.color_format = k4a.EPixelFormat.BGRA32
    config.color_resolution = k4a.EColorResolution.RES_1080P
    config.depth_mode = k4a.EDepthMode.NFOV_UNBINNED
    k4a.device_start_cameras(config)

    while True:
        capture = k4a.capture_t()
        #if k4a.device_get_capture(k4a.timeout_t(K4A_WAIT_INFINITE), capture) == k4a.K4A_WAIT_RESULT.SUCCEEDED:
            # Verarbeiten Sie die erfassten Daten hier
            # Zum Beispiel: Farbbilder, Tiefenbilder usw. erhalten

    k4a.device_close()

if __name__ == "__main__":
    main()
