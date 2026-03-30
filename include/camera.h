#ifndef CAMERA_H
#define CAMERA_H

// Header pour la Camera
#include <Arduino.h>
#include "esp_camera.h"

void server_routes_camera();

int camera_set_parameter(sensor_t *s, const char *variable, int val, bool save);
void camera_load_settings(sensor_t *s, camera_config_t *config);
void encodeP();

#endif
