// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "SPIFFS.h"
#include "variables.h"

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "camera.h"
#include <Preferences.h>
#include <time.h>

#ifdef SDCARD
#include "SDMMC.h"
#endif

#define NUM_CAMERA 01

extern Preferences preferences_nvs;
extern AsyncWebServer server;
#include "html/index_ov2640.h"
#include "html/index_ov3660.h"

/*extern const char index_ov2640_html[];
extern const size_t index_ov2640_html_len;

extern const char index_ov3660_html[];
extern const size_t index_ov3660_html_len;*/

//extern const char index_ov5640_html[];
//extern const size_t index_ov5640_html_len;


#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char *TAG = "camera_httpd";
#endif

// Face Detection will not work on boards without (or with disabled) PSRAM
#ifdef BOARD_HAS_PSRAM
//#define CONFIG_ESP_FACE_DETECT_ENABLED 1
// Face Recognition takes upward from 15 seconds per frame on chips other than ESP32S3
// Makes no sense to have it enabled for them
    #if CONFIG_IDF_TARGET_ESP32S3
    #define CONFIG_ESP_FACE_RECOGNITION_ENABLED 0
    #define CONFIG_ESP_FACE_DETECT_ENABLED 0
    #else
    #define CONFIG_ESP_FACE_RECOGNITION_ENABLED 0
    #define CONFIG_ESP_FACE_DETECT_ENABLED 0
    #endif
#else
    #define CONFIG_ESP_FACE_DETECT_ENABLED 0
    #define CONFIG_ESP_FACE_RECOGNITION_ENABLED 0
#endif

#if CONFIG_ESP_FACE_DETECT_ENABLED

    #include <vector>
    #include "human_face_detect_msr01.hpp"
    #include "human_face_detect_mnp01.hpp"

    #define TWO_STAGE 1 /*<! 1: detect by two-stage which is more accurate but slower(with keypoints). */
                        /*<! 0: detect by one-stage which is less accurate but faster(without keypoints). */

    #if CONFIG_ESP_FACE_RECOGNITION_ENABLED
    #include "face_recognition_tool.hpp"
    #include "face_recognition_112_v1_s16.hpp"
    #include "face_recognition_112_v1_s8.hpp"

    #define QUANT_TYPE 0 //if set to 1 => very large firmware, very slow, reboots when streaming...

    #define FACE_ID_SAVE_NUMBER 7
    #endif

    #define FACE_COLOR_WHITE 0x00FFFFFF
    #define FACE_COLOR_BLACK 0x00000000
    #define FACE_COLOR_RED 0x000000FF
    #define FACE_COLOR_GREEN 0x0000FF00
    #define FACE_COLOR_BLUE 0x00FF0000
    #define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
    #define FACE_COLOR_CYAN (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
    #define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)
#endif

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
int led_duty = 0;
bool isStreaming = false;
#ifdef CONFIG_LED_LEDC_LOW_SPEED_MODE
#define CONFIG_LED_LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#else
#define CONFIG_LED_LEDC_SPEED_MODE LEDC_HIGH_SPEED_MODE
#endif
#endif

typedef struct
{
    size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

#if CONFIG_ESP_FACE_DETECT_ENABLED

static int8_t detection_enabled = 0;

// #if TWO_STAGE
// static HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
// static HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
// #else
// static HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
// #endif

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
static int8_t recognition_enabled = 0;
static int8_t is_enrolling = 0;

#if QUANT_TYPE
    // S16 model
    FaceRecognition112V1S16 recognizer;
#else
    // S8 model
    FaceRecognition112V1S8 recognizer;
#endif
#endif

#endif

typedef struct
{
    size_t size;  //number of values used for filtering
    size_t index; //current value index
    size_t count; //value count
    int sum;
    int *values; //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size)
{
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if (!filter->values)
    {
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
static int ra_filter_run(ra_filter_t *filter, int value)
{
    if (!filter->values)
    {
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size)
    {
        filter->count++;
    }
    return filter->sum / filter->count;
}
#endif

#if CONFIG_ESP_FACE_DETECT_ENABLED
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
static void rgb_print(fb_data_t *fb, uint32_t color, const char *str)
{
    fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(fb_data_t *fb, uint32_t color, const char *format, ...)
{
    char loc_buf[64];
    char *temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if (len >= sizeof(loc_buf))
    {
        temp = (char *)malloc(len + 1);
        if (temp == NULL)
        {
            return 0;
        }
    }
    vsnprintf(temp, len + 1, format, arg);
    va_end(arg);
    rgb_print(fb, color, temp);
    if (len > 64)
    {
        free(temp);
    }
    return len;
}
#endif
static void draw_face_boxes(fb_data_t *fb, std::list<dl::detect::result_t> *results, int face_id)
{
    int x, y, w, h;
    uint32_t color = FACE_COLOR_YELLOW;
    if (face_id < 0)
    {
        color = FACE_COLOR_RED;
    }
    else if (face_id > 0)
    {
        color = FACE_COLOR_GREEN;
    }
    if(fb->bytes_per_pixel == 2){
        //color = ((color >> 8) & 0xF800) | ((color >> 3) & 0x07E0) | (color & 0x001F);
        color = ((color >> 16) & 0x001F) | ((color >> 3) & 0x07E0) | ((color << 8) & 0xF800);
    }
    int i = 0;
    for (std::list<dl::detect::result_t>::iterator prediction = results->begin(); prediction != results->end(); prediction++, i++)
    {
        // rectangle box
        x = (int)prediction->box[0];
        y = (int)prediction->box[1];
        w = (int)prediction->box[2] - x + 1;
        h = (int)prediction->box[3] - y + 1;
        if((x + w) > fb->width){
            w = fb->width - x;
        }
        if((y + h) > fb->height){
            h = fb->height - y;
        }
        fb_gfx_drawFastHLine(fb, x, y, w, color);
        fb_gfx_drawFastHLine(fb, x, y + h - 1, w, color);
        fb_gfx_drawFastVLine(fb, x, y, h, color);
        fb_gfx_drawFastVLine(fb, x + w - 1, y, h, color);
#if TWO_STAGE
        // landmarks (left eye, mouth left, nose, right eye, mouth right)
        int x0, y0, j;
        for (j = 0; j < 10; j+=2) {
            x0 = (int)prediction->keypoint[j];
            y0 = (int)prediction->keypoint[j+1];
            fb_gfx_fillRect(fb, x0, y0, 3, 3, color);
        }
#endif
    }
}

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
static int run_face_recognition(fb_data_t *fb, std::list<dl::detect::result_t> *results)
{
    std::vector<int> landmarks = results->front().keypoint;
    int id = -1;

    Tensor<uint8_t> tensor;
    tensor.set_element((uint8_t *)fb->data).set_shape({fb->height, fb->width, 3}).set_auto_free(false);

    int enrolled_count = recognizer.get_enrolled_id_num();

    if (enrolled_count < FACE_ID_SAVE_NUMBER && is_enrolling){
        id = recognizer.enroll_id(tensor, landmarks, "", true);
        ESP_LOGI(TAG, "Enrolled ID: %d", id);
        rgb_printf(fb, FACE_COLOR_CYAN, "ID[%u]", id);
    }

    face_info_t recognize = recognizer.recognize(tensor, landmarks);
    if(recognize.id >= 0){
        rgb_printf(fb, FACE_COLOR_GREEN, "ID[%u]: %.2f", recognize.id, recognize.similarity);
    } else {
        rgb_print(fb, FACE_COLOR_RED, "Intruder Alert!");
    }
    return recognize.id;
}
#endif
#endif

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
void enable_led(bool en)
{ // Turn LED On or Off
    int duty = en ? led_duty : 0;
    if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY))
    {
        duty = CONFIG_LED_MAX_INTENSITY;
    }
    ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
    ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
    ESP_LOGI(TAG, "Set LED intensity to %d", duty);
}
#endif

static void bmp_handler(AsyncWebServerRequest *request)
{
    camera_fb_t *fb = NULL;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint64_t fr_start = esp_timer_get_time();
#endif
    fb = esp_camera_fb_get();
    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        request->send(500, "text/plain", "Camera capture failed");
        return;
    }

    uint8_t * buf = NULL;
    size_t buf_len = 0;
    bool converted = frame2bmp(fb, &buf, &buf_len);
    esp_camera_fb_return(fb);
    if(!converted){
        ESP_LOGE(TAG, "BMP Conversion failed");
        request->send(500, "text/plain", "BMP Conversion failed");
        return;
    }
    
    AsyncWebServerResponse *response = request->beginResponse(200, "image/x-windows-bmp", buf, buf_len);
    response->addHeader("Content-Disposition", "inline; filename=capture.bmp");
    response->addHeader("Access-Control-Allow-Origin", "*");
    
    char ts[32];
    snprintf(ts, 32, "%lld", esp_timer_get_time() / 1000);
    response->addHeader("X-Timestamp", ts);
    
    request->send(response);
    free(buf);
    
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint64_t fr_end = esp_timer_get_time();
    ESP_LOGI(TAG, "BMP: %llums, %uB", (uint64_t)((fr_end - fr_start) / 1000), buf_len);
#endif
}

static void capture_handler(AsyncWebServerRequest *request)
{
    camera_fb_t *fb = NULL;
    uint8_t *buf = NULL;
    size_t buf_len = 0;
    int buf_format = 0;  // Save format before returning fb
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    int64_t fr_start = esp_timer_get_time();
#endif

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    enable_led(true);
    vTaskDelay(150 / portTICK_PERIOD_MS);
    fb = esp_camera_fb_get();
    enable_led(false);
#else
    fb = esp_camera_fb_get();
#endif

    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        request->send(500, "text/plain", "Camera capture failed");
        return;
    }

    buf_format = fb->format;  // Save format before returning fb

#if CONFIG_ESP_FACE_DETECT_ENABLED
    size_t out_len, out_width, out_height;
    uint8_t *out_buf;
    bool s;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    bool detected = false;
#endif
    int face_id = 0;
    if (!detection_enabled || fb->width > 400)
    {
#endif
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
        size_t fb_len = 0;
#endif
        if (fb->format == PIXFORMAT_JPEG)
        {
            #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                        fb_len = fb->len;
            #endif
            buf = fb->buf;
            buf_len = fb->len;
        }
        else
        {
            frame2jpg(fb, 80, &buf, &buf_len);
            #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                        fb_len = buf_len;
            #endif
        }
        esp_camera_fb_return(fb);
        
        AsyncWebServerResponse *response = request->beginResponse(200, "image/jpeg", buf, buf_len);
        response->addHeader("Content-Disposition", "inline; filename=capture.jpg");
        response->addHeader("Access-Control-Allow-Origin", "*");
        
        char ts[32];
        snprintf(ts, 32, "%lld", esp_timer_get_time() / 1000);
        response->addHeader("X-Timestamp", ts);
        
        
        request->send(response);
        
        if (buf && buf_format != PIXFORMAT_JPEG) {
            free(buf);
        }
        #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                int64_t fr_end = esp_timer_get_time();
                ESP_LOGI(TAG, "JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
        #endif
        return;
        #if CONFIG_ESP_FACE_DETECT_ENABLED
            }

            if (fb->format == PIXFORMAT_RGB565
        #if CONFIG_ESP_FACE_RECOGNITION_ENABLED
            && !recognition_enabled
        #endif
            ){
        #if TWO_STAGE
                HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
                HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
                std::list<dl::detect::result_t> &candidates = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
                std::list<dl::detect::result_t> &results = s2.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3}, candidates);
        #else
                HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
                std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
        #endif
                if (results.size() > 0) {
                    fb_data_t rfb;
                    rfb.width = fb->width;
                    rfb.height = fb->height;
                    rfb.data = fb->buf;
                    rfb.bytes_per_pixel = 2;
                    rfb.format = FB_RGB565;
        #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                    detected = true;
        #endif
                    draw_face_boxes(&rfb, &results, face_id);
                }
                s = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 90, &buf, &buf_len);
                esp_camera_fb_return(fb);
            } else
            {
                out_len = fb->width * fb->height * 3;
                out_width = fb->width;
                out_height = fb->height;
                out_buf = (uint8_t*)malloc(out_len);
                if (!out_buf) {
                    ESP_LOGE(TAG, "out_buf malloc failed");
                    esp_camera_fb_return(fb);
                    request->send(500, "text/plain", "Memory allocation failed");
                    return;
                }
                s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
                esp_camera_fb_return(fb);
                if (!s) {
                    free(out_buf);
                    ESP_LOGE(TAG, "to rgb888 failed");
                    request->send(500, "text/plain", "Format conversion failed");
                    return;
                }

                fb_data_t rfb;
                rfb.width = out_width;
                rfb.height = out_height;
                rfb.data = out_buf;
                rfb.bytes_per_pixel = 3;
                rfb.format = FB_BGR888;

        #if TWO_STAGE
                HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
                HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
                std::list<dl::detect::result_t> &candidates = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
                std::list<dl::detect::result_t> &results = s2.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3}, candidates);
        #else
                HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
                std::list<dl::detect::result_t> &results = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
        #endif

                if (results.size() > 0) {
        #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                    detected = true;
        #endif
        #if CONFIG_ESP_FACE_RECOGNITION_ENABLED
                    if (recognition_enabled) {
                        face_id = run_face_recognition(&rfb, &results);
                    }
        #endif
                    draw_face_boxes(&rfb, &results, face_id);
                }

                s = fmt2jpg(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, &buf, &buf_len);
                free(out_buf);
            }

            if (!s) {
                ESP_LOGE(TAG, "JPEG compression failed");
                request->send(500, "text/plain", "JPEG compression failed");
                return;
            }

            AsyncWebServerResponse *response = request->beginResponse(200, "image/jpeg", buf, buf_len);
            response->addHeader("Content-Disposition", "inline; filename=capture.jpg");
            response->addHeader("Access-Control-Allow-Origin", "*");
            
            char ts[32];
            snprintf(ts, 32, "%lld", esp_timer_get_time() / 1000);
            response->addHeader("X-Timestamp", ts);
            
            request->send(response);
            
            if (buf) {
                free(buf);
            }
        #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
            int64_t fr_end = esp_timer_get_time();
            ESP_LOGI(TAG, "FACE: %uB %ums %s%d", (uint32_t)(buf_len), (uint32_t)((fr_end - fr_start) / 1000), detected ? "DETECTED " : "", face_id);
        #endif
        #endif
}

static void capture_handler_SD(AsyncWebServerRequest *request)
{
    camera_fb_t *fb = NULL;
    uint8_t *buf = NULL;
    size_t buf_len = 0;
    int buf_format = 0;  // Save format before returning fb
    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
        int64_t fr_start = esp_timer_get_time();
    #endif

    #ifdef CONFIG_LED_ILLUMINATOR_ENABLED
        enable_led(true);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        fb = esp_camera_fb_get();
        enable_led(false);
    #else
        fb = esp_camera_fb_get();
    #endif

    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        request->send(500, "text/plain", "Camera capture failed");
        return;
    }

    buf_format = fb->format;  // Save format before returning fb

    #if CONFIG_ESP_FACE_DETECT_ENABLED
        size_t out_len, out_width, out_height;
        uint8_t *out_buf;
        bool s;
    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
        bool detected = false;
    #endif
        int face_id = 0;
        if (!detection_enabled || fb->width > 400)
        {
    #endif
    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
            size_t fb_len = 0;
    #endif
    if (fb->format == PIXFORMAT_JPEG)
    {
        #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                    fb_len = fb->len;
        #endif
        buf = fb->buf;
        buf_len = fb->len;
    }
    else
    {
        frame2jpg(fb, 80, &buf, &buf_len);
        #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                    fb_len = buf_len;
        #endif
    }
    esp_camera_fb_return(fb);
    
    // Sauvegarde sur SD
    #ifdef SDCARD
        lectureHeure(); // Met à jour timeinfo
        char dir_path[32];
        char file_path[64];
        char base_name[32];
        int year = timeinfo.tm_year + 1900;
        int month = timeinfo.tm_mon + 1;
        int day = timeinfo.tm_mday;
        int hour = timeinfo.tm_hour;
        int min = timeinfo.tm_min;
        int sec = timeinfo.tm_sec;
        fs::FS &fs = SD_MMC; // Assuming fs is SD_MMC

        // Créer le répertoire /YYYY/MM/DD
        snprintf(dir_path, sizeof(dir_path), "/%04d/%02d/%02d", year, month, day);
        //Serial.printf("testinf directory: %s\n", dir_path);
        if (!fs.exists(dir_path)) {
            Serial.printf("Creating directory: %s\n", dir_path);
            createDir(fs, dir_path);
        }

        // Vérifier si le fichier existe, incrémenter les secondes si nécessaire
        int current_sec = sec;
        int current_min = min;
        int current_hour = hour;
        int attempts = 0;
        
        do {
            snprintf(file_path, sizeof(file_path), "%s/C%02d_%04d%02d%02d_%02d%02d%02d.jpg", dir_path, NUM_CAMERA, year, month, day, current_hour, current_min, current_sec);
            
            if (!fs.exists(file_path)) {
                break; // Fichier n'existe pas, on peut l'utiliser
            }
            
            // Incrémenter les secondes pour le prochain test
            current_sec++;
            if (current_sec >= 60) {
                current_sec = 0;
                current_min++;
                if (current_min >= 60) {
                    current_min = 0;
                    current_hour++;
                    if (current_hour >= 24) {
                        current_hour = 0;
                    }
                }
            }
            attempts++;
        } while (attempts < 100); // Limite à 100 pour éviter boucle infinie

        // Sauvegarder le fichier
        uint8_t result = writeFile(fs, file_path, buf, buf_len);
        if (result == 0) {
            //ESP_LOGI(TAG, "Image saved to %s", file_path);
            request->send(200, "text/plain", "Image saved to SD card");
        } else {
            ESP_LOGE(TAG, "Failed to save image to %s", file_path);
            request->send(500, "text/plain", "Failed to save image to SD card");
        }
    #else
        request->send(500, "text/plain", "SD card not available");
    #endif
    
    if (buf && buf_format != PIXFORMAT_JPEG) {
        free(buf);
    }
    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
            int64_t fr_end = esp_timer_get_time();
            ESP_LOGI(TAG, "JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    #endif
    return;
}

static void stream_handler(AsyncWebServerRequest *request)
{
    camera_fb_t *fb = NULL;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    
    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    #ifdef CONFIG_LED_ILLUMINATOR_ENABLED
        enable_led(true);
        isStreaming = true;
    #endif

    // For streaming with AsyncWebServer, we collect frames and send as multipart
    // Limiting to a few frames per connection for memory efficiency
    std::vector<uint8_t> response_data;
    
    // Add initial boundary
    String boundary = _STREAM_BOUNDARY;
    for (char c : boundary) {
        response_data.push_back(c);
    }
    
    int loop_count = 0;
    int max_frames = 5;  // Limit frames per request
    
    while (loop_count < max_frames)
    {
        fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            break;
        }
        
        if (fb->format != PIXFORMAT_JPEG)
        {
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            esp_camera_fb_return(fb);
            fb = NULL;
            if (!jpeg_converted)
            {
                ESP_LOGE(TAG, "JPEG compression failed");
                break;
            }
        }
        else
        {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }
        
        // Add part header
        char part_buf[128];
        size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len, (long)esp_timer_get_time() / 1000000, (long)(esp_timer_get_time() % 1000000));
        for (size_t i = 0; i < hlen; i++) {
            response_data.push_back(part_buf[i]);
        }
        
        // Add image data
        for (size_t i = 0; i < _jpg_buf_len; i++) {
            response_data.push_back(_jpg_buf[i]);
        }
        
        // Add boundary
        for (char c : boundary) {
            response_data.push_back(c);
        }
        
        if (fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        frame_time /= 1000;
        
        #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
                ESP_LOGI(TAG, "MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)",
                        (uint32_t)(_jpg_buf_len),
                        (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
                        avg_frame_time, 1000.0 / avg_frame_time);
        #endif
        
        last_frame = fr_end;
        loop_count++;
        
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }

    #ifdef CONFIG_LED_ILLUMINATOR_ENABLED
        isStreaming = false;
        enable_led(false);
    #endif

    // Send the complete response
    AsyncWebServerResponse *response = request->beginResponse(200, _STREAM_CONTENT_TYPE, response_data.data(), response_data.size());
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("X-Framerate", "60");
    request->send(response);
}

static void cmd_handler(AsyncWebServerRequest *request)
{
    char variable[32] = {0};
    char value[32] = {0};
    
    // Extract parameters from query string
    if (request->hasParam("var") && request->hasParam("val")) {
        strncpy(variable, request->getParam("var")->value().c_str(), sizeof(variable) - 1);
        strncpy(value, request->getParam("val")->value().c_str(), sizeof(value) - 1);
    } else {
        request->send(404, "text/plain", "Missing parameters");
        return;
    }

    int val = atoi(value);
    ESP_LOGI(TAG, "%s = %d", variable, val);
    sensor_t *s = esp_camera_sensor_get();
    int res = camera_set_parameter(s, variable, val, true);

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    if (res < 0 && !strcmp(variable, "led_intensity")) {
        res = 0;
        led_duty = val;
        if (isStreaming)
            enable_led(true);
        preferences_nvs.putInt("led_intensity", val);
    }
#endif

#if CONFIG_ESP_FACE_DETECT_ENABLED
    if (!strcmp(variable, "face_detect")) {
        res = 0;
        detection_enabled = val;
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
        if (!detection_enabled) {
            recognition_enabled = 0;
        }
#endif
    }
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
    else if (!strcmp(variable, "face_enroll")){
        res = 0;
        is_enrolling = !is_enrolling;
        ESP_LOGI(TAG, "Enrolling: %s", is_enrolling?"true":"false");
    }
    else if (!strcmp(variable, "face_recognize")) {
        res = 0;
        recognition_enabled = val;
        if (recognition_enabled) {
            detection_enabled = val;
        }
    }
#endif
#endif
    if (res < 0) {
        ESP_LOGI(TAG, "Unknown command: %s", variable);
        request->send(500, "text/plain", "Command failed");
        return;
    }

    request->send(200, "text/plain", "OK");
}


static bool camera_is_valid_framesize(framesize_t v)
{
#if defined(FRAMESIZE_QQVGA) && defined(FRAMESIZE_UXGA)
    return (v >= FRAMESIZE_QQVGA && v <= FRAMESIZE_UXGA);
#else
    // Sans macro, on accepte les valeurs 0..12 (OV2640 classique)
    return (v >= 0 && v <= 12);
#endif
}

static bool camera_is_valid_pixformat(pixformat_t v)
{
    return (v == PIXFORMAT_RGB565 || v == PIXFORMAT_YUV422 || v == PIXFORMAT_GRAYSCALE || v == PIXFORMAT_JPEG || v == PIXFORMAT_RGB888);
}

static int camera_get_validated_int(const char *key, int value, int minValue, int maxValue)
{
    int stored = preferences_nvs.getInt(key, value);
    if (stored < minValue || stored > maxValue) {
        stored = value;
        preferences_nvs.putInt(key, stored);
    }
    return stored;
}

int camera_set_parameter(sensor_t *s, const char *variable, int val, bool save)
{
    int res = -1;

    if (!strcmp(variable, "framesize")) {
        if (s->pixformat == PIXFORMAT_JPEG) {
            res = s->set_framesize(s, (framesize_t)val);
        }
    }
    else if (!strcmp(variable, "quality"))
        res = s->set_quality(s, val);
    else if (!strcmp(variable, "contrast"))
        res = s->set_contrast(s, val);
    else if (!strcmp(variable, "brightness"))
        res = s->set_brightness(s, val);
    else if (!strcmp(variable, "saturation"))
        res = s->set_saturation(s, val);
    else if (!strcmp(variable, "gainceiling"))
        res = s->set_gainceiling(s, (gainceiling_t)val);
    else if (!strcmp(variable, "colorbar"))
        res = s->set_colorbar(s, val);
    else if (!strcmp(variable, "awb"))
        res = s->set_whitebal(s, val);
    else if (!strcmp(variable, "agc"))
        res = s->set_gain_ctrl(s, val);
    else if (!strcmp(variable, "aec"))
        res = s->set_exposure_ctrl(s, val);
    else if (!strcmp(variable, "hmirror"))
        res = s->set_hmirror(s, val);
    else if (!strcmp(variable, "vflip"))
        res = s->set_vflip(s, val);
    else if (!strcmp(variable, "awb_gain"))
        res = s->set_awb_gain(s, val);
    else if (!strcmp(variable, "agc_gain"))
        res = s->set_agc_gain(s, val);
    else if (!strcmp(variable, "aec_value"))
        res = s->set_aec_value(s, val);
    else if (!strcmp(variable, "aec2"))
        res = s->set_aec2(s, val);
    else if (!strcmp(variable, "dcw"))
        res = s->set_dcw(s, val);
    else if (!strcmp(variable, "bpc"))
        res = s->set_bpc(s, val);
    else if (!strcmp(variable, "wpc"))
        res = s->set_wpc(s, val);
    else if (!strcmp(variable, "raw_gma"))
        res = s->set_raw_gma(s, val);
    else if (!strcmp(variable, "lenc"))
        res = s->set_lenc(s, val);
    else if (!strcmp(variable, "special_effect"))
        res = s->set_special_effect(s, val);
    else if (!strcmp(variable, "wb_mode"))
        res = s->set_wb_mode(s, val);
    else if (!strcmp(variable, "ae_level"))
        res = s->set_ae_level(s, val);
    else if (!strcmp(variable, "xclk"))
        res = s->set_xclk(s, LEDC_TIMER_0, val);
    else if (!strcmp(variable, "reg")) {
        // Expects val composed: (reg<<16)|(mask<<8)|value ; re-créer si nécessaire
        // on ne sait pas exactement comment parser depuis URL. nvs n'est pas recommandé.
        res = -2;
    }
    else if (!strcmp(variable, "pll"))
        res = s->set_pll(s, 0, 0, 0, 0, 0, 0, 0, 0);
    else if (!strcmp(variable, "res_raw"))
        res = -2;

    if ((res >= 0) && save) {
        // Même clé que variable (et/ou préfixe identique) pour retrouver facilement
        preferences_nvs.putInt(variable, val);
    }

    return res;
}


void camera_load_settings(sensor_t *s, camera_config_t *config)
{
    if (config) {
        bool config_valid = true;

        uint32_t stored_framesize = preferences_nvs.getUInt("framesize", config->frame_size);
        if (!camera_is_valid_framesize((framesize_t)stored_framesize)) {
            config_valid = false;
        }

        uint32_t stored_pixformat = preferences_nvs.getUInt("pixel_format", config->pixel_format);
        if (!camera_is_valid_pixformat((pixformat_t)stored_pixformat)) {
            config_valid = false;
        }

        if (config_valid) {
            // lecture de  paramètres config lus et validés
            config->frame_size = (framesize_t)stored_framesize;
            config->pixel_format = (pixformat_t)stored_pixformat;

            uint32_t stored_grab_mode = preferences_nvs.getUInt("grab_mode", config->grab_mode);
            config->grab_mode = (camera_grab_mode_t)stored_grab_mode;

            uint32_t stored_jpeg_quality = preferences_nvs.getUInt("jpeg_quality", config->jpeg_quality);
            if (stored_jpeg_quality > 63) {
                stored_jpeg_quality = config->jpeg_quality;
            }
            config->jpeg_quality = stored_jpeg_quality;

            uint32_t stored_fb_count = preferences_nvs.getUInt("fb_count", config->fb_count);
            if (stored_fb_count < 1 || stored_fb_count > 4) {
                stored_fb_count = config->fb_count;
            }
            config->fb_count = stored_fb_count;

        } else {
            // Si l'un des paramètres clés est invalide, écriture des valeurs par défaut dans NVS
            preferences_nvs.putUInt("framesize", config->frame_size);
            preferences_nvs.putUInt("pixel_format", config->pixel_format);
            preferences_nvs.putUInt("grab_mode", config->grab_mode);
            preferences_nvs.putUInt("jpeg_quality", config->jpeg_quality);
            preferences_nvs.putUInt("fb_count", config->fb_count);
        }
    }

    if (s) {
        // Exemples de validation sensor (a minima 2 variables) et relecture
        int brightness = camera_get_validated_int("brightness", s->status.brightness, -2, 2);
        camera_set_parameter(s, "brightness", brightness, false);

        int contrast = camera_get_validated_int("contrast", s->status.contrast, -2, 2);
        camera_set_parameter(s, "contrast", contrast, false);

        int saturation = camera_get_validated_int("saturation", s->status.saturation, -2, 2);
        camera_set_parameter(s, "saturation", saturation, false);

        int quality = camera_get_validated_int("quality", s->status.quality, 1, 63);
        camera_set_parameter(s, "quality", quality, false);

        // autres paramètres, récupérés et appliqués s'ils existent
        camera_set_parameter(s, "gainceiling", camera_get_validated_int("gainceiling", s->status.gainceiling, 0, 6), false);
        camera_set_parameter(s, "awb", camera_get_validated_int("awb", s->status.awb, 0, 1), false);
        camera_set_parameter(s, "agc", camera_get_validated_int("agc", s->status.agc, 0, 1), false);
        camera_set_parameter(s, "aec", camera_get_validated_int("aec", s->status.aec, 0, 1), false);
        camera_set_parameter(s, "hmirror", camera_get_validated_int("hmirror", s->status.hmirror, 0, 1), false);
        camera_set_parameter(s, "vflip", camera_get_validated_int("vflip", s->status.vflip, 0, 1), false);

        // faire attention : framesize doit être appliqué après config init si possible
        int fs = camera_get_validated_int("framesize", s->status.framesize, 0, 12);
        if (s->pixformat == PIXFORMAT_JPEG) {
            camera_set_parameter(s, "framesize", fs, false);
        }

        // Appliquer res_raw si sauvegardé
        if (preferences_nvs.isKey("res_raw_startX")) {
            int startX = preferences_nvs.getInt("res_raw_startX", 0);
            int startY = preferences_nvs.getInt("res_raw_startY", 0);
            int endX = preferences_nvs.getInt("res_raw_endX", 0);
            int endY = preferences_nvs.getInt("res_raw_endY", 0);
            int offsetX = preferences_nvs.getInt("res_raw_offsetX", 0);
            int offsetY = preferences_nvs.getInt("res_raw_offsetY", 0);
            int totalX = preferences_nvs.getInt("res_raw_totalX", 0);
            int totalY = preferences_nvs.getInt("res_raw_totalY", 0);
            int outputX = preferences_nvs.getInt("res_raw_outputX", 0);
            int outputY = preferences_nvs.getInt("res_raw_outputY", 0);
            int scale = preferences_nvs.getInt("res_raw_scale", 0);
            int binning = preferences_nvs.getInt("res_raw_binning", 0);
            s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
        }
    }
}


static int print_reg(char * p, sensor_t * s, uint16_t reg, uint32_t mask){
    return sprintf(p, "\"0x%x\":%u,", reg, s->get_reg(s, reg, mask));
}

static void status_handler(AsyncWebServerRequest *request)
{
    static char json_response[1024];

    sensor_t *s = esp_camera_sensor_get();
    char *p = json_response;
    *p++ = '{';

    if(s->id.PID == OV5640_PID || s->id.PID == OV3660_PID){
        for(int reg = 0x3400; reg < 0x3406; reg+=2){
            p+=print_reg(p, s, reg, 0xFFF);//12 bit
        }
        p+=print_reg(p, s, 0x3406, 0xFF);

        p+=print_reg(p, s, 0x3500, 0xFFFF0);//16 bit
        p+=print_reg(p, s, 0x3503, 0xFF);
        p+=print_reg(p, s, 0x350a, 0x3FF);//10 bit
        p+=print_reg(p, s, 0x350c, 0xFFFF);//16 bit

        for(int reg = 0x5480; reg <= 0x5490; reg++){
            p+=print_reg(p, s, reg, 0xFF);
        }

        for(int reg = 0x5380; reg <= 0x538b; reg++){
            p+=print_reg(p, s, reg, 0xFF);
        }

        for(int reg = 0x5580; reg < 0x558a; reg++){
            p+=print_reg(p, s, reg, 0xFF);
        }
        p+=print_reg(p, s, 0x558a, 0x1FF);//9 bit
    } else if(s->id.PID == OV2640_PID){
        p+=print_reg(p, s, 0xd3, 0xFF);
        p+=print_reg(p, s, 0x111, 0xFF);
        p+=print_reg(p, s, 0x132, 0xFF);
    }

    p += sprintf(p, "\"xclk\":%u,", s->xclk_freq_hz / 1000000);
    p += sprintf(p, "\"pixformat\":%u,", s->pixformat);
    p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p += sprintf(p, "\"quality\":%u,", s->status.quality);
    p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p += sprintf(p, "\"awb\":%u,", s->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p += sprintf(p, "\"aec\":%u,", s->status.aec);
    p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p += sprintf(p, "\"agc\":%u,", s->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    p += sprintf(p, ",\"led_intensity\":%u", led_duty);
#else
    p += sprintf(p, ",\"led_intensity\":%d", -1);
#endif
#if CONFIG_ESP_FACE_DETECT_ENABLED
    p += sprintf(p, ",\"face_detect\":%u", detection_enabled);
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
    p += sprintf(p, ",\"face_enroll\":%u,", is_enrolling);
    p += sprintf(p, "\"face_recognize\":%u", recognition_enabled);
#endif
#endif
    *p++ = '}';
    *p++ = 0;
    
    request->send(200, "application/json", json_response);
}

static void xclk_handler(AsyncWebServerRequest *request)
{
    int xclk = 0;
    
    if (request->hasParam("xclk")) {
        xclk = atoi(request->getParam("xclk")->value().c_str());
    } else {
        request->send(404, "text/plain", "Missing xclk parameter");
        return;
    }
    
    ESP_LOGI(TAG, "Set XCLK: %d MHz", xclk);

    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_xclk(s, LEDC_TIMER_0, xclk);
    if (res) {
        request->send(500, "text/plain", "XCLK setting failed");
        return;
    }

    request->send(200, "text/plain", "OK");
}

static void reg_handler(AsyncWebServerRequest *request)
{
    int reg = 0, mask = 0, val = 0;
    
    if (!request->hasParam("reg") || !request->hasParam("mask") || !request->hasParam("val")) {
        request->send(404, "text/plain", "Missing parameters");
        return;
    }
    
    reg = atoi(request->getParam("reg")->value().c_str());
    mask = atoi(request->getParam("mask")->value().c_str());
    val = atoi(request->getParam("val")->value().c_str());
    
    ESP_LOGI(TAG, "Set Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, val);

    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_reg(s, reg, mask, val);
    if (res) {
        request->send(500, "text/plain", "Register setting failed");
        return;
    }

    request->send(200, "text/plain", "OK");
}

static void greg_handler(AsyncWebServerRequest *request)
{
    int reg = 0, mask = 0;
    
    if (!request->hasParam("reg") || !request->hasParam("mask")) {
        request->send(404, "text/plain", "Missing parameters");
        return;
    }
    
    reg = atoi(request->getParam("reg")->value().c_str());
    mask = atoi(request->getParam("mask")->value().c_str());
    
    sensor_t *s = esp_camera_sensor_get();
    int res = s->get_reg(s, reg, mask);
    if (res < 0) {
        request->send(500, "text/plain", "Register read failed");
        return;
    }
    ESP_LOGI(TAG, "Get Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, res);

    char buffer[20];
    const char * val = itoa(res, buffer, 10);
    request->send(200, "text/plain", val);
}

static int parse_get_var(AsyncWebServerRequest *request, const char *key, int def)
{
    if (request->hasParam(key)) {
        return atoi(request->getParam(key)->value().c_str());
    }
    return def;
}

static void pll_handler(AsyncWebServerRequest *request)
{
    int bypass = parse_get_var(request, "bypass", 0);
    int mul = parse_get_var(request, "mul", 0);
    int sys = parse_get_var(request, "sys", 0);
    int root = parse_get_var(request, "root", 0);
    int pre = parse_get_var(request, "pre", 0);
    int seld5 = parse_get_var(request, "seld5", 0);
    int pclken = parse_get_var(request, "pclken", 0);
    int pclk = parse_get_var(request, "pclk", 0);

    ESP_LOGI(TAG, "Set Pll: bypass: %d, mul: %d, sys: %d, root: %d, pre: %d, seld5: %d, pclken: %d, pclk: %d", bypass, mul, sys, root, pre, seld5, pclken, pclk);
    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_pll(s, bypass, mul, sys, root, pre, seld5, pclken, pclk);
    if (res) {
        request->send(500, "text/plain", "PLL setting failed");
        return;
    }

    request->send(200, "text/plain", "OK");
}

static void win_handler(AsyncWebServerRequest *request)
{
    int startX = parse_get_var(request, "sx", 0);
    int startY = parse_get_var(request, "sy", 0);
    int endX = parse_get_var(request, "ex", 0);
    int endY = parse_get_var(request, "ey", 0);
    int offsetX = parse_get_var(request, "offx", 0);
    int offsetY = parse_get_var(request, "offy", 0);
    int totalX = parse_get_var(request, "tx", 0);
    int totalY = parse_get_var(request, "ty", 0);
    int outputX = parse_get_var(request, "ox", 0);
    int outputY = parse_get_var(request, "oy", 0);
    bool scale = parse_get_var(request, "scale", 0) == 1;
    bool binning = parse_get_var(request, "binning", 0) == 1;

    ESP_LOGI(TAG, "Set Window: Start: %d %d, End: %d %d, Offset: %d %d, Total: %d %d, Output: %d %d, Scale: %u, Binning: %u", startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
    if (res) {
        request->send(500);
        return;
    }

    // Sauvegarde des paramètres res_raw en NVS
    preferences_nvs.putInt("res_raw_startX", startX);
    preferences_nvs.putInt("res_raw_startY", startY);
    preferences_nvs.putInt("res_raw_endX", endX);
    preferences_nvs.putInt("res_raw_endY", endY);
    preferences_nvs.putInt("res_raw_offsetX", offsetX);
    preferences_nvs.putInt("res_raw_offsetY", offsetY);
    preferences_nvs.putInt("res_raw_totalX", totalX);
    preferences_nvs.putInt("res_raw_totalY", totalY);
    preferences_nvs.putInt("res_raw_outputX", outputX);
    preferences_nvs.putInt("res_raw_outputY", outputY);
    preferences_nvs.putInt("res_raw_scale", scale);
    preferences_nvs.putInt("res_raw_binning", binning);

    request->send(200);
}

static void index_handler(AsyncWebServerRequest *request)
{
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        if (s->id.PID == OV3660_PID) {
           request->send(200, "text/html", (const uint8_t*)index_ov3660_html, index_ov3660_html_len);
        } else {
            request->send(200, "text/html", (const uint8_t*)index_ov2640_html, index_ov2640_html_len);
        }  
    } else {
        ESP_LOGE(TAG, "Camera sensor not found");
        request->send(500);
    }
}


void server_routes_camera()
{
    ra_filter_init(&ra_filter, 20);

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
    recognizer.set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr");
    // load ids from flash partition
    recognizer.set_ids_from_flash();
#endif

    // Register camera routes with AsyncWebServer
    server.on("/cam", HTTP_GET, [](AsyncWebServerRequest *request){
        index_handler(request);
    });

    server.on("/Cstatus", HTTP_GET, [](AsyncWebServerRequest *request){
        status_handler(request);
    });

    server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request){
        cmd_handler(request);
    });

    server.on("/capture", HTTP_GET, [](AsyncWebServerRequest *request){
        capture_handler(request);
    });

    server.on("/captureSD", HTTP_GET, [](AsyncWebServerRequest *request){
        capture_handler_SD(request);
    });

    server.on("/stream", HTTP_GET, [](AsyncWebServerRequest *request){
        stream_handler(request);
    });

    server.on("/bmp", HTTP_GET, [](AsyncWebServerRequest *request){
        bmp_handler(request);
    });

    server.on("/xclk", HTTP_GET, [](AsyncWebServerRequest *request){
        xclk_handler(request);
    });

    server.on("/reg", HTTP_GET, [](AsyncWebServerRequest *request){
        reg_handler(request);
    });

    server.on("/greg", HTTP_GET, [](AsyncWebServerRequest *request){
        greg_handler(request);
    });

    server.on("/pll", HTTP_GET, [](AsyncWebServerRequest *request){
        pll_handler(request);
    });

    server.on("/resolution", HTTP_GET, [](AsyncWebServerRequest *request){
        win_handler(request);
    });

    ESP_LOGI(TAG, "Camera server routes registered");
}