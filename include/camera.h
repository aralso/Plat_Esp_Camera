#ifndef CAMERA_H
#define CAMERA_H

// Header pour la Camera
#include <Arduino.h>
#include "esp_camera.h"

void server_routes_camera();

int camera_set_parameter(sensor_t *s, const char *variable, int val, bool save);
void camera_load_settings(sensor_t *s, camera_config_t *config);
void encodeP();

// Size/codec helpers
// New signatures: each function provides the other two values via output parameters.
// - size_to_code: input width (pixels), outputs framesize (cam_size) and code (index), returns code
// - camcode_to_code: input framesize (cam_code), outputs code and representative width, returns code
// - code_to_size: input code, outputs representative width and framesize, returns code
uint8_t size_to_code(uint16_t size, framesize_t &cam_size, uint8_t &code);
uint8_t camcode_to_code(framesize_t cam_size, uint8_t &code, uint16_t &rep_width);
uint8_t code_to_size(uint8_t code, uint16_t &rep_width, framesize_t &cam_size);

// JPEG compression mapping helpers
// tx_compjpg_to_code: input txJpg(percent), outputs txCam (camera jpeg_quality) and code
uint8_t tx_compjpg_to_code(uint8_t txJpg, uint8_t &txCam, uint8_t &code);
// inverse: from txCam to representative txJpg and code
uint8_t txcam_to_compjpg(uint8_t txCam, uint8_t &txJpg, uint8_t &code);
// inverse: from code to txJpg and txCam
uint8_t code_to_compjpg(uint8_t code, uint8_t &txJpg, uint8_t &txCam);

uint8_t nbIm_to_code (uint8_t nb_images);

// Global code mapping helpers
// Convert a triplet (images_code, size_code, comp_code) into a single global char code
char triplet_to_global(uint8_t images_code, uint8_t size_code, uint8_t comp_code);
// Parse a global char code into the triplet; returns true if found
bool global_to_triplet(char global_code, uint8_t &images_code, uint8_t &size_code, uint8_t &comp_code);


#endif
