idf_component_register(
    SRCS
        "Appli.cpp"
        "Ard_ESP_Core.cpp"
        "cpu_load.cpp"
        "SDMMC.cpp"
        "variables.cpp"
        "variables_current_sensor_quality.cpp"

        "Camera/Cam.cpp"
        "Camera/app_httpd.cpp"

        "encoder/encoder.cpp"
        "encoder/esp_jpg_decode.cpp"
        "encoder/jpge.cpp"
        "encoder/mjpegw.c"
        "encoder/tjpgd.cpp"
        "encoder/to_bmp.cpp"
        "encoder/yuv.cpp"

        "LPC/lpc.cpp"

    INCLUDE_DIRS
        "."
        "Camera"
        "encoder"
        "LPC"
)