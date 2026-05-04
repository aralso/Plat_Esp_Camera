#include <Arduino.h>

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "variables.h"

#include "esp_camera.h"
#include "camera.h"
#include "../lpc/lpc.h"

// issu de :     zhuhai-esp /  ESP32-S3-Goouuu-Cam

//#include "header.h"
//#include "esp_jpg_decode.h"

#include "SDMMC.h"
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
    Serial.printf("PS RAM not Found \n");

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


uint8_t setup_camera() {
  uint8_t res = initCamera();
  if (res == 0) {
    Serial.print("Camera Ready! Use 'http://");
    return 0;
  }
  else
  {
    Serial.println("Camera initialization failed");
    return 1;
  }
}

void encodeP()
{
  if (sdcard_ok) 
  {
    lpc_settings_t settings = {
      800,    // width
      600,    // height
      30,     // quality
      1,      // frame_count
      1,       // frequency
    };
    //encode_lpc(settings);
    encode_lpc2(settings, nullptr, 0, "/sd/lpc_test.jpg");
  }
}

bool getJpegSize(uint8_t *buf, size_t len, uint16_t &w, uint16_t &h) {
    for (size_t i = 0; i < len - 9; i++) {
        if (buf[i] == 0xFF && buf[i+1] == 0xC0) {
            h = (buf[i+5] << 8) + buf[i+6];
            w = (buf[i+7] << 8) + buf[i+8];
            if ((w>100) && (w<2000) && (h>50) && (h<2000)) 
              return true;
            else
              return false;
        }
    }
    return false;
}

uint8_t reduc_image(fs::FS &fs, uint8_t* jpg_buf, size_t fileSize, const char *path1, uint16_t newsize, uint16_t quality)
{
    // recuperation de l w et h de l'image source.
    uint16_t w = 0, h = 0;

    if (!getJpegSize(jpg_buf, fileSize, w, h)) {
        Serial.println("Failed to read JPEG size");
        free(jpg_buf);
        return 3;
    }

    Serial.printf("Image size: %i x %i\n", w, h);

    // --- 2. décoder JPEG → RGB ---
    uint8_t* rgb_buf = NULL;
    size_t rgb_len;

 		// alloc RGB888
		rgb_len = (size_t)w * (size_t)h * 3 * sizeof(uint8_t);
		rgb_buf = (uint8_t*)malloc(rgb_len);
		if (rgb_buf) 
			memset(rgb_buf, 0, rgb_len);
    else {
      free(jpg_buf);
      return 9;
    }

    if (!fmt2rgb888(jpg_buf, fileSize, PIXFORMAT_JPEG, rgb_buf)) {
        Serial.println("JPEG decode failed");
        free(jpg_buf);
        free(rgb_buf);
        return 4;
    }
    free(jpg_buf);

    

    // --- 3. calcul nouvelle taille ---
    int new_w = newsize;
    int new_h = newsize * h/w;

    if (new_w > w)
    {
        Serial.printf("ce n'est pas une reduction: actuel:%d new: %d\n", w, new_w);
        free(rgb_buf);
        return 5;
    }

    // --- 4. allocation buffer réduit ---
    uint8_t* rgb_small = (uint8_t*)malloc(new_w * new_h * 3);
    if (!rgb_small) {
        Serial.println("Malloc small failed");
        free(rgb_buf);
        return 6;
    }

      // --- 5. resize simple (nearest neighbor) ---
    for (int y = 0; y < new_h; y++) {
        vTaskDelay(1);
        for (int x = 0; x < new_w; x++) {

            int src_x = x * w / new_w;
            int src_y = (h - 1) - (y * h / new_h);  // 🔥 inversion ici

            memcpy(
                &rgb_small[(y * new_w + x) * 3],
                &rgb_buf[(src_y * w + src_x) * 3],
                3
            );
        }
    }

    free(rgb_buf);

    // --- 6. encoder JPEG ---
    uint8_t* jpg_out = NULL;
    size_t jpg_len = 0;

    if (!fmt2jpg(rgb_small, new_w * new_h * 3, new_w, new_h,
                 PIXFORMAT_RGB888, quality, &jpg_out, &jpg_len)) {

        Serial.println("JPEG encode failed");
        free(rgb_small);
        return 7;
    }

    free(rgb_small);

    // --- 7. créer nom sortie ---
    String newPath = String(path1);
    int dot = newPath.lastIndexOf('.');

    if (dot > 0) {
        newPath = newPath.substring(0, dot) + "_" + String(newsize) + ".jpg";
    } else {
        newPath += "_" + String(newsize) + ".jpg";
    }

    uint8_t res = sauve_image(fs, newPath.c_str(), jpg_out, jpg_len);

    return res;
}
