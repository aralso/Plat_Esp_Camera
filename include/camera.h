#ifndef CAMERA_H
#define CAMERA_H

// Header pour la Camera
#include <Arduino.h>
#include "esp_camera.h"

void server_routes_camera();

int camera_set_parameter(sensor_t *s, const char *variable, int val, bool save);
void camera_load_settings(sensor_t *s, camera_config_t *config);
void encodeP();

// Map a camera sensor quality value (txCam, e.g. 63..4) to an interpolated JPEG quality percent (0..100)
uint8_t txcam_to_compjpg_interp(uint8_t txCam);



#endif
