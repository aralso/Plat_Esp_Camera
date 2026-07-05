#include <Arduino.h>

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "variables.h"

#include "esp_camera.h"
#include "camera.h"
#include "../lpc/lpc.h"

// size/code helpers moved to end of file
// (size_to_code and code_to_size are defined at the end of this TU)

// issu de :     zhuhai-esp /  ESP32-S3-Goouuu-Cam

//#include "header.h"
//#include "esp_jpg_decode.h"

#include "SDMMC.h"
#include "camera_pins.h"


// nb images : 1(1) 2(2) 3(4) 4(8) 5(16) 6(32) 7(64)
// Size 0:160 1:QVGA(320),2:HVGA(480) 3:VGA(640), 4:SVGA(800), 5:XGA(1024), 6:SXGA(1280) 7:1600 8:2048
// Compress jpg  : moins bon 0(0%-63) 1(5%-55) 2(10%-50) 3(15%-40) 4(20%-30) 5(30%-20) 6(50%-12) 7(60%-8) 8(80%-6) 9(100%-4) meilleur
// global Nb_im+size+qual : A:1+320+1 C:2+320+2 E:3+480+3 H:640+3 K:640+4 N:800+4 Q:800+5 T:800+6 Z:4+1024+6
// CA-260618-201223-E-323.jpg  .avi  .lpc


// ex :
// Framesize 0:QQVGA(160) 1:HQVGA(240),2:QVGA(320),3:CIF(400),4:HVGA(480) 5:VGA(640), 6:SVGA(800), 7:XGA(1024), 8:HD(1280), 9:SXGA(1280)
// Quali cam    : meilleur 0(4) 1(10) 2(14) 3(20) 4(30) 5(50) 6(60) moins bon
// Compress jpg  : moins bon 0 1(10) 2(14) 3(20) 4(30) 5(50) 6(60) 100  meilleur
// nom fichier : C01-026-F5-Q3-T0-201001-000001

 uint8_t cap_nb_images;
 uint8_t cap_interval_dsec;
 uint8_t cap_size;
 uint8_t cap_jpg_comp;
 uint8_t im_x_debut;
 uint8_t im_x_fin;
 uint8_t im_y_debut;
 uint8_t im_y_fin;

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
  config.frame_size = FRAMESIZE_SXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  // Use latest frame grab mode to avoid serving old frames queued in driver
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 4;
  config.fb_count = 1;
  // Nota : frame_size initial et jpeg_quality initial definisse la taille maximale qu'on pourra charger

  // Charger les valeurs précédemment enregistrées dans NVS
  camera_load_settings(nullptr, &config);

  // Force pixel format to JPEG to avoid expensive on-the-fly conversions
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    Serial.printf("PS RAM Found [%d]\n", ESP.getPsramSize());
    config.jpeg_quality = 4;
    // Use more frame buffers when PSRAM is available to avoid contention between stream and captures
  config.fb_count = 4; // increase for robustness (was 3)
    config.grab_mode = CAMERA_GRAB_LATEST; // prefer the most recent frame
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
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
    Serial.println("Camera Ready! Use 'http://cam");
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

// --- Helpers moved here: size/code mapping and JPEG compression mappings ---

// Size mapping helpers (codes -> widths) with framesize mapping
typedef struct { uint16_t width; framesize_t cam_size; } size_entry_t;
static const size_entry_t size_table[] = {
    {96,   FRAMESIZE_96X96},
    {160,  FRAMESIZE_QQVGA},
    {176,  FRAMESIZE_QCIF},
    {240,  FRAMESIZE_HQVGA},
    {240,  FRAMESIZE_240X240},
    {320,  FRAMESIZE_QVGA},
    {400,  FRAMESIZE_CIF},
    {480,  FRAMESIZE_HVGA},
    {640,  FRAMESIZE_VGA},
    {720,  FRAMESIZE_P_HD},
    {800,  FRAMESIZE_SVGA},
    {864,  FRAMESIZE_P_3MP},
    {1024, FRAMESIZE_XGA},
    {1080, FRAMESIZE_P_FHD},
    {1280, FRAMESIZE_HD},
    {1280, FRAMESIZE_SXGA},
    {1600, FRAMESIZE_UXGA},
    {1920, FRAMESIZE_FHD},
    {2048, FRAMESIZE_QXGA},
    {2560, FRAMESIZE_QHD},
    {2560, FRAMESIZE_WQXGA},
    {2560, FRAMESIZE_QSXGA}
};
static const size_t size_table_count = sizeof(size_table) / sizeof(size_table[0]);

// Given an image width (size), return the code (index) and output cam_size via reference.
// Uses the lower-threshold rule: choose the largest entry width <= size.
uint8_t size_to_code(uint16_t size, framesize_t &cam_size, uint8_t &code)
{
    uint8_t idx = 0;
    for (size_t i = 0; i < size_table_count; ++i) {
        if (size >= size_table[i].width) idx = (uint8_t)i;
        else break;
    }
    code = idx;
    cam_size = size_table[idx].cam_size;
    return code;
}

// Given a framesize (cam_size), return the code and representative width via reference.
// If exact framesize not found in the table, the function returns the closest match by search (first match not found -> last).
uint8_t camsize_to_code(framesize_t cam_size, uint8_t &code, uint16_t &rep_width)
{
    for (size_t i = 0; i < size_table_count; ++i) {
        if (size_table[i].cam_size == cam_size) {
            code = (uint8_t)i;
            rep_width = size_table[i].width;
            return code;
        }
    }
    // Not found: return last entry
    code = (uint8_t)(size_table_count - 1);
    rep_width = size_table[code].width;
    return code;
}

// Given a code (index), return representative width and framesize (clamped to table bounds).
uint8_t code_to_size(uint8_t in_code, uint16_t &rep_width, framesize_t &cam_size)
{
    uint8_t c = in_code;
    if ((size_t)c >= size_table_count) c = (uint8_t)(size_table_count - 1);
    rep_width = size_table[c].width;
    cam_size = size_table[c].cam_size;
    return c;
}

// JPEG compression mapping table
// Entry: code, tc_comp_jpg (percent representative), tx_comp_cam (camera jpeg_quality)
struct comp_entry_t { uint8_t code; uint8_t tc_comp_jpg; uint8_t tx_comp_cam; };

static const comp_entry_t comp_table[] = {
    {0, 0,   63},
    {1, 5,   55},
    {2, 10,  50},
    {3, 15,  40},
    {4, 20,  30},
    {5, 30,  20},
    {6, 50,  12},
    {7, 60,   8},
    {8, 80,   6},
    {9, 100,  4}
};
static const size_t comp_table_count = sizeof(comp_table) / sizeof(comp_table[0]);

// Convert txJpg (percent, 0..100) to code and txCam (jpeg_quality):
// - chooses the largest tc_comp_jpg threshold <= txJpg (lower-threshold rule)
// - returns the code (also written to 'code') and sets txCam to the mapped camera quality
uint8_t tx_compjpg_to_code(uint8_t txJpg, uint8_t &txCam, uint8_t &code)
{
    uint8_t c = 0;
    for (size_t i = 0; i < comp_table_count; ++i) {
        if (txJpg >= comp_table[i].tc_comp_jpg) c = (uint8_t)i;
        else break;
    }
    code = c;
    txCam = comp_table[c].tx_comp_cam;
    return code;
}

// Inverse: given txCam (camera jpeg_quality), choose the nearest comp_table entry by absolute difference
// and return the representative txJpg and code
uint8_t txcam_to_compjpg(uint8_t txCam, uint8_t &txJpg, uint8_t &code)
{
    int best_idx = 0;
    int best_diff = abs((int)comp_table[0].tx_comp_cam - (int)txCam);
    for (size_t i = 1; i < comp_table_count; ++i) {
        int diff = abs((int)comp_table[i].tx_comp_cam - (int)txCam);
        if (diff < best_diff) {
            best_diff = diff;
            best_idx = (int)i;
        }
    }
    code = (uint8_t)best_idx;
    txJpg = comp_table[best_idx].tc_comp_jpg;
    return code;
}

// Inverse: given code, return txJpg and txCam (clamped)
uint8_t code_to_compjpg(uint8_t in_code, uint8_t &txJpg, uint8_t &txCam)
{
    uint8_t c = in_code;
    if ((size_t)c >= comp_table_count) c = (uint8_t)(comp_table_count - 1);
    txJpg = comp_table[c].tc_comp_jpg;
    txCam = comp_table[c].tx_comp_cam;
    return c;
}
