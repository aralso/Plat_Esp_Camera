#include <Arduino.h>

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "variables.h"

#include "esp_camera.h"
#include "camera.h"

// issu de :     zhuhai-esp /  ESP32-S3-Goouuu-Cam

#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"



uint8_t inline initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_SVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Charger les valeurs précédemment enregistrées dans NVS
  camera_load_settings(nullptr, &config);

  if (psramFound()) {
    Serial.printf("PS RAM Found [%d]\n", ESP.getPsramSize());
    config.jpeg_quality = 10;
    config.fb_count = 2; // 1 seule image 2;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // evite de lire 2° image trop viteCAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return 1;
  }
  sensor_t *s = esp_camera_sensor_get();
  // restaurer les réglages stockés sur NVS
  camera_load_settings(s, nullptr);

  //s->set_vflip(s, 1);      // flip it back
  //s->set_brightness(s, 1); // up the brightness just a bit
  //s->set_saturation(s, 0); // lower the saturation
  return 0;
}


void setup_camera() {
  initCamera();
  Serial.print("Camera Ready! Use 'http://");
}

