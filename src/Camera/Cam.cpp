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


 uint8_t cap_nb_images;
 uint8_t cap_interval_dsec;
 uint8_t cap_size;
 uint8_t cap_jpg_comp;
 uint8_t im_x_debut;
 uint8_t im_x_fin;
 uint8_t im_y_debut;
 uint8_t im_y_fin;
 uint8_t type_cam;   // 0:non def, 1:OV3660  2:0V2640

 bool fmt2rgb888_scaled(const uint8_t *src_buf, size_t src_len, pixformat_t format, uint8_t * rgb_buf, jpg_scale_t scale, int *width, int *height);

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
uint8_t code_to_nbIm (uint8_t code);

// Global code mapping helpers
// Convert a triplet (images_code, size_code, comp_code) into a single global char code
char triplet_to_global(uint8_t images_code, uint8_t size_code, uint8_t comp_code);
// Parse a global char code into the triplet; returns true if found
bool global_to_triplet(char global_code, uint8_t &images_code, uint8_t &size_code, uint8_t &comp_code);


// Size mapping helpers (codes -> widths) with framesize mapping
typedef struct { uint16_t width; uint16_t height; framesize_t cam_code; uint8_t code; } size_entry_t;
static const size_entry_t size_table[] = {
    {96,   96,   FRAMESIZE_96X96,  0},
    {160,  120,  FRAMESIZE_QQVGA,  0},
    {176,  144,  FRAMESIZE_QCIF,   0},
    {240,  176,  FRAMESIZE_HQVGA,  1},
    {240,  240,  FRAMESIZE_240X240,1},
    {320,  240,  FRAMESIZE_QVGA,   1},
    {400,  296,  FRAMESIZE_CIF,    1},
    {480,  320,  FRAMESIZE_HVGA,   2},
    {640,  480,  FRAMESIZE_VGA,    3},
    {720,  480,  FRAMESIZE_P_HD,   3},
    {800,  600,  FRAMESIZE_SVGA,   4},
    {864,  648,  FRAMESIZE_P_3MP,  4},
    {1024, 768,  FRAMESIZE_XGA,    5},
    {1080, 720,  FRAMESIZE_P_FHD,  5},
    {1280, 720,  FRAMESIZE_HD,     5},
    {1280, 1024, FRAMESIZE_SXGA,   6},
    {1600, 1200, FRAMESIZE_UXGA,   7},
    {1920, 1080, FRAMESIZE_FHD,    7},
    {2048, 1536, FRAMESIZE_QXGA,   8},
    {2560, 1440, FRAMESIZE_QHD,    8},
    {2560, 1600, FRAMESIZE_WQXGA,  8},
    {2560, 1920, FRAMESIZE_QSXGA,  9}
};
static const size_t size_table_count = sizeof(size_table) / sizeof(size_table[0]);

// Return width/height for a framesize_t by looking up size_table[]
// Sets width and height via reference parameters. If not found, sets both to 0.
static inline void framesize_to_wh(framesize_t fs, uint16_t &width, uint16_t &height)
{
    for (size_t i = 0; i < size_table_count; ++i) {
        if (size_table[i].cam_code == fs) {
            width = size_table[i].width;
            height = size_table[i].height;
            return;
        }
    }
    // Not found: set defaults
    width = 0;
    height = 0;
}


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
  if (s != NULL) {
        if (s->id.PID == OV3660_PID) type_cam=1;
        else type_cam=2; // OV2640
  }

  //camera_set_parameter(s, "framesize", fs, false);
  s->set_framesize(s, (framesize_t)9);

  //s->set_vflip(s, 1);      // flip it back
  //s->set_brightness(s, 1); // up the brightness just a bit
  //s->set_saturation(s, 0); // lower the saturation

  // handler / cam dans app_httpd.cpp : index_handler

  return 0;
}

uint8_t configCamera()
{
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("Failed to get camera sensor");
    return 1;
  }

  // Set the frame size and JPEG quality based on the global variables codes
  uint16_t width;
  uint16_t height;
  framesize_t cam_size; // camera
  code_to_size(cap_size, width, cam_size);
  framesize_to_wh(cam_size, width, height);
  //Serial.printf("Configuring camera: cap_size code=%d, cam=%d, width=%i height=%i\n", cap_size, cam_size, width, height);
  s->set_framesize(s, cam_size);

  uint8_t txjpg;
  uint8_t txcam;
  code_to_compjpg(cap_jpg_comp, txjpg, txcam);
  s->set_quality(s, txcam);

  // Additional camera settings can be configured here if needed
  uint16_t x_S = ((uint32_t) width * im_x_debut / 100) & 0xFFF0;
  uint16_t x_E = ((uint32_t) width * im_x_fin / 100) & 0xFFF0;
  uint16_t y_S = ((uint32_t) height * im_y_debut / 100) & 0xFFF0;
  uint16_t y_E = ((uint32_t) height * im_y_fin / 100) & 0xFFF0;
  if ((x_E > x_S) && (y_E > y_S) && (x_E <= width) && (y_E <= height))
  {  
    Serial.printf("Camera windowing set to: x_S=%d, x_E=%d, y_S=%d, y_E=%d\n", x_S, x_E, y_S, y_E);
    if ((im_x_debut) || (im_x_fin < 100) || (im_y_debut) || (im_y_fin < 100))
    {
        int res = 0;
        if (type_cam==2)  // OV2640
        {
       //res = s->set_res_raw(s, x_S, y_S, x_E, y_E, 0, 0, width, height, width, height, 0, 0);
        //  int res = s->set_res_raw(s, 0,0,0,0, im_x_debut, im_y_debut, 
        //    im_x_fin-im_x_debut, im_y_fin-im_y_debut, im_x_fin-im_x_debut, im_y_fin-im_y_debut,0, 0);
        // ov2640 :
        // setWindow(start_x, 0, 0, 0, offset_x, offset_y, total_x, total_y, output_x, output_y, false, false, function(code, txt){
       //Set Window: Start: 0 0, End: 0 0, Offset: 400 300, Total: 800 600, Output: 320 240, Scale: 0, Binning: 0
        }
        if (type_cam==1) // OV3660
        {
       // ov3660 : 
       //     setWindow(start_x, start_y, end_x, end_y, offset_x, offset_y, total_x, total_y, output_x, output_y, scaling, binning, function(code, txt){
          //int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
        }
        Serial.printf("Camera windowing applied %d\n", res);
    }
  }
  return 0;
}

uint8_t setup_camera() {
  uint8_t res = initCamera();
  if (res == 0) {
    Serial.println("Camera Ready! Use 'http://cam");
  }
  else
  {
    Serial.println("Camera initialization failed");
    return 1;
  }
  return configCamera();
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
    printMemoryStatus();
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
    size_t rgb_len = 0;

    // Decide on a decode scale to reduce memory usage when possible.
    // Use the JPEG decoder built-in scaling: JPG_SCALE_NONE, JPG_SCALE_2X, JPG_SCALE_4X, JPG_SCALE_8X
    jpg_scale_t chosen_scale = JPG_SCALE_NONE;

    // Determine target width after scaling. Choose the largest scale that still produces
    // an image larger or equal to the desired reduced width to preserve quality.
    if (newsize <= w / 8) {
        chosen_scale = JPG_SCALE_8X;
    } else if (newsize <= w / 4) {
        chosen_scale = JPG_SCALE_4X;
    } else if (newsize <= w / 2) {
        chosen_scale = JPG_SCALE_2X;
    } else {
        chosen_scale = JPG_SCALE_NONE;
    }

    int dec_w = 0, dec_h = 0;

    // Calculate approximate decoded dimensions to size the RGB buffer
    switch (chosen_scale) {
        case JPG_SCALE_8X: dec_w = w / 8; dec_h = h / 8; break;
        case JPG_SCALE_4X: dec_w = w / 4; dec_h = h / 4; break;
        case JPG_SCALE_2X: dec_w = w / 2; dec_h = h / 2; break;
        default: dec_w = w; dec_h = h; break;
    }

    // allocate RGB888 for the scaled image
    rgb_len = (size_t)dec_w * (size_t)dec_h * 3;
    rgb_buf = (uint8_t*)malloc(rgb_len);
    if (!rgb_buf) {
        free(jpg_buf);
        return 9;
    }
    memset(rgb_buf, 0, rgb_len);
    printMemoryStatus();

    // Decode using the scaled decoder (new function). This avoids allocating full-size RGB on large JPEGs.
    if (!fmt2rgb888_scaled(jpg_buf, fileSize, PIXFORMAT_JPEG, rgb_buf, chosen_scale, &dec_w, &dec_h)) {
        Serial.println("JPEG scaled decode failed");
        free(jpg_buf);
        free(rgb_buf);
        return 4;
    }
    free(jpg_buf);

    // dec_w/dec_h contain the actual decoded dimensions; recompute new_h relative to decoded height
    // Scale-up or -down as needed when resizing to the requested newsize
    int src_w = dec_w;
    int src_h = dec_h;

    

    // --- 3. calcul nouvelle taille ---
    int new_w = newsize;
    int new_h = (int)((long)newsize * src_h / src_w);

    if (new_w > src_w)
    {
        Serial.printf("ce n'est pas une reduction: actuel:%d new: %d\n", src_w, new_w);
        free(rgb_buf);
        return 5;
    }

    // --- 4. allocation buffer réduit ---
    uint8_t* rgb_small = (uint8_t*)malloc((size_t)new_w * (size_t)new_h * 3);
    if (!rgb_small) {
        Serial.println("Malloc small failed");
        free(rgb_buf);
        return 6;
    }
    printMemoryStatus();

    // --- 5. resize simple (nearest neighbor) ---
    for (int y = 0; y < new_h; y++) {
        vTaskDelay(1);
        for (int x = 0; x < new_w; x++) {

            int src_x = x * src_w / new_w;
            int src_y = (src_h - 1) - (y * src_h / new_h);  // preserve original vertical orientation logic

            memcpy(
                &rgb_small[(y * new_w + x) * 3],
                &rgb_buf[(src_y * src_w + src_x) * 3],
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



// Given an image width (size), return the code (index) and output cam_size via reference.
// Uses the lower-threshold rule: choose the largest entry width <= size.
uint8_t size_to_code(uint16_t size, framesize_t &cam_code, uint8_t &code)
{
    uint8_t idx = 0;
    for (size_t i = 0; i < size_table_count; ++i) {
        if (size >= size_table[i].width) idx = (uint8_t)i;
        else break;
    }
    code = size_table[idx].code;
    cam_code = size_table[idx].cam_code;
    return 0;
}

// Given a framesize (cam_code), return the code and representative width via reference.
// If exact framesize not found in the table, the function returns the closest match by search (first match not found -> last).
uint8_t camcode_to_code(framesize_t cam_code, uint8_t &code, uint16_t &rep_width)
{
    uint8_t c = cam_code;
    if ((size_t)c >= size_table_count) c = (uint8_t)(size_table_count - 1);
    rep_width = size_table[c].width;
    code = size_table[c].code;
    return 0;
}

// Given a code (index), return representative width and framesize (clamped to table bounds).
uint8_t code_to_size(uint8_t code, uint16_t &rep_width, framesize_t &cam_code)
{
    for (size_t i = 0; i < size_table_count; ++i) {
        if (size_table[i].code == code) {
            cam_code = size_table[i].cam_code;
            rep_width = size_table[i].width;
            //Serial.printf("code:%i cam_code:%i i:%d width:%i\n", code, cam_code, i, rep_width);
            return 0;
        }
    }
    // Not found: return VGA
    cam_code = size_table[8].cam_code;
    rep_width = size_table[8].width;
    return 1;
}

// JPEG compression mapping table
// Entry: code, tc_comp_jpg (percent representative), tx_comp_cam (camera jpeg_quality)
struct comp_entry_t { uint8_t code; uint8_t tx_comp_jpg; uint8_t tx_comp_cam; };

static const comp_entry_t comp_table[] = {
    {0, 5,   63},
    {1, 12,  55},
    {2, 20,  50},
    {3, 28,  45},
    {4, 40,  40},
    {5, 50,  30},
    {6, 60,  20},
    {7, 70,  12},
    {8, 82,   7},
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
        if (txJpg >= comp_table[i].tx_comp_jpg) c = (uint8_t)i;
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
    txJpg = comp_table[best_idx].tx_comp_jpg;
    return code;
}

// Inverse: given code, return txJpg and txCam (clamped)
uint8_t code_to_compjpg(uint8_t in_code, uint8_t &txJpg, uint8_t &txCam)
{
    uint8_t c = in_code;
    if ((size_t)c >= comp_table_count) c = (uint8_t)(comp_table_count - 1);
    txJpg = comp_table[c].tx_comp_jpg;
    txCam = comp_table[c].tx_comp_cam;
    return c;
}

uint8_t nbIm_to_code (uint8_t nb_imag)
{
    uint8_t p = 1;

    while (nb_imag > 1 && p < 8)
    {
        p++;
        nb_imag >>= 1;
    }

    return p;
}

uint8_t code_to_nbIm (uint8_t code)
{
    uint8_t nb_imag = 1;

    while (code > 1 && nb_imag < 64)
    {
        code--;
        nb_imag <<= 1;
    }

    return nb_imag;
}

// ---------- Global code mapping ----------
// Table entry mapping a single-char global code to the triplet (images_code, size_code, comp_code)
typedef struct { char gc; uint8_t images; uint8_t size; uint8_t comp; } global_entry_t;

// Unified static table mapping single-char global code to the triplet (images_code, size_code, comp_code)
// A..Z entries, A fixed to 100 (1,0,0), Z fixed to 789 (7,8,9), intermediates unique.
static global_entry_t global_table[] = {
    { 'A', 1, 0, 0 }, // 100 (fixed)
    { 'B', 1, 0, 1 },
    { 'C', 1, 0, 2 },
    { 'D', 1, 1, 3 },
    { 'E', 1, 2, 3 },
    { 'F', 1, 3, 3 },
    { 'G', 1, 4, 3 },
    { 'H', 1, 4, 5 },
    { 'I', 1, 4, 6 },
    { 'J', 1, 4, 7 },
    { 'K', 1, 5, 7 },
    { 'L', 1, 6, 7 },
    { 'M', 5, 6, 8 },
    { 'N', 5, 7, 8 },
    { 'O', 6, 7, 8 },
    { 'P', 6, 7, 9 },
    { 'Q', 6, 8, 9 },
    { 'R', 7, 0, 1 },
    { 'S', 7, 1, 2 },
    { 'T', 7, 2, 3 },
    { 'U', 7, 3, 4 },
    { 'V', 7, 4, 5 },
    { 'W', 7, 5, 6 },
    { 'X', 7, 6, 7 },
    { 'Y', 7, 7, 8 },
    { 'Z', 7, 8, 9 }  // 789 (fixed)
};
static const size_t global_table_count = sizeof(global_table)/sizeof(global_table[0]);


static const global_entry_t* global_to_triplet_det(char gc)
{
    for (size_t i = 0; i < global_table_count; ++i) {
        if (global_table[i].gc == gc) return &global_table[i];
    }
    return NULL;
}

bool global_to_triplet(char global_code, uint8_t &images_code, uint8_t &size_code, uint8_t &comp_code)
{
    const global_entry_t* e = global_to_triplet_det(global_code);
    if (!e) return false;
    images_code = e->images;
    size_code = e->size;
    comp_code = e->comp;
    return true;
}

char triplet_to_global(uint8_t images_code, uint8_t size_code, uint8_t comp_code)
{
    Serial.printf("triplet_to_global: searching for images_code=%d, size_code=%d, comp_code=%d\n", images_code, size_code, comp_code);
    // Exact match first
    for (size_t i = 0; i < global_table_count; ++i)
    {
        if (global_table[i].images == images_code && global_table[i].size == size_code && global_table[i].comp == comp_code)
        {
            Serial.printf("triplet_to_global: exact match found at index %d, global_code=%c\n", (int)i, global_table[i].gc);
            return global_table[i].gc;
        }
    }

    // No exact match: choose the entry with the smallest signed sum difference
    // diff_signed = (g.images - images_code) + (g.size - size_code) + (g.comp - comp_code)
    // Primary key: minimal abs(diff_signed)
    // Tie-breaker: minimal sum of absolute differences

    int best_idx = -1;
    int best_abs_signed = INT32_MAX;
    int best_abs_sum = INT32_MAX;

    for (size_t i = 0; i < global_table_count; ++i) {
        const global_entry_t &g = global_table[i];
        int diff_images = (int)g.images - (int)images_code;
        int diff_size = (int)g.size - (int)size_code;
        int diff_comp = (int)g.comp - (int)comp_code;
        int signed_sum = diff_images + diff_size + diff_comp;
        int abs_signed = (signed_sum >= 0) ? signed_sum : -signed_sum;
        int abs_sum = abs(diff_images) + abs(diff_size) + abs(diff_comp);

        if (abs_signed < best_abs_signed) {
            best_abs_signed = abs_signed;
            best_abs_sum = abs_sum;
            best_idx = (int)i;
        } else if (abs_signed == best_abs_signed) {
            if (abs_sum < best_abs_sum) {
                best_abs_sum = abs_sum;
                best_idx = (int)i;
            }
        }
        Serial.printf("triplet_to_global: checking %c: diff_images=%d, diff_size=%d, diff_comp=%d, signed_sum=%d, abs_signed=%d, abs_sum=%d\n",
            g.gc, diff_images, diff_size, diff_comp, signed_sum, abs_signed, abs_sum);
    }

    if (best_idx >= 0) return global_table[best_idx].gc;
    return 'Z';
}

