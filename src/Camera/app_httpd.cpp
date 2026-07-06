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
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#ifdef SDCARD
#include "SDMMC.h"
#endif

#define NUM_CAMERA 01

extern Preferences preferences_nvs;
extern AsyncWebServer server;
#include "html/index_ov2640.h"
#include "html/index_ov3660.h"
#include "mjpegw.h"  // used to build MJPEG AVI files on SD card (mjpeg frames)

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

// AVI capture session state used to perform non-blocking captures across multiple
// event loop invocations. The HTTP handler posts EVENT_PRISE_VIDEO and returns;
// the event loop calls prise_video() which calls capture_avi_background(), and
// after each frame a FreeRTOS one-shot timer re-posts EVENT_PRISE_VIDEO until
// the requested number of frames has been captured.
static struct {
    bool active;
    struct mjpegw_context *avi;
    int frames_total;
    int frames_captured;
    int frames_remaining;
    int width;
    int height;
    int quality;
    char file_path[128];
    TimerHandle_t timer;
} avi_session = {0};

// Timer callback posted when it's time to capture the next frame. Runs in the
// timer daemon task context; it simply posts EVENT_PRISE_VIDEO to the eventQueue.
static void avi_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    systeme_eve_t evt = { EVENT_PRISE_VIDEO, 0 };
    if (eventQueue) {
        BaseType_t ok = xQueueSend(eventQueue, &evt, 0);
        if (ok != pdTRUE) {
            ESP_LOGE(TAG, "avi_timer_cb: failed to post EVENT_PRISE_VIDEO");
        }
    }
}


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
    // Guard: ensure camera sensor initialized before calling fb_get
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "Camera not initialized (bmp_handler)");
        request->send(500, "text/plain", "Camera not initialized");
        return;
    }

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
    // Schedule delayed free to avoid use-after-free while AsyncWebServer is sending
    if (buf) {
        esp_timer_handle_t once_timer;
        esp_timer_create_args_t targs = {
            .callback = [](void* arg){ free(arg); },
            .arg = buf,
            .name = "free_buf"
        };
        if (esp_timer_create(&targs, &once_timer) == ESP_OK) {
            esp_timer_start_once(once_timer, 2000000);
        } else {
            free(buf);
        }
    }
    
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

    // Guard: ensure camera sensor initialized before calling fb_get
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "Camera not initialized (capture_handler)");
        request->send(500, "text/plain", "Camera not initialized");
        return;
    }

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    ESP_LOGI(TAG, "capture_handler(): enabling LED and capturing");
    enable_led(true);
    vTaskDelay(150 / portTICK_PERIOD_MS);
    fb = esp_camera_fb_get();
    enable_led(false);
#else
    ESP_LOGI(TAG, "capture : capturing (no LED)");
    fb = esp_camera_fb_get();
#endif

    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed: fb==NULL. freeHeap=%u psramFound=%d", (unsigned)ESP.getFreeHeap(), (int)psramFound());
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
            // Copy JPEG buffer because fb->buf will be returned to the driver
            buf_len = fb->len;
            buf = (uint8_t*)malloc(buf_len);
            if (!buf) {
                esp_camera_fb_return(fb);
                ESP_LOGE(TAG, "Malloc failed for JPEG buffer copy");
                request->send(500, "text/plain", "Memory allocation failed");
                return;
            }
            memcpy(buf, fb->buf, buf_len);
        }
        else
        {
            frame2jpg(fb, 80, &buf, &buf_len);
            #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                        fb_len = buf_len;
            #endif
        }

        int64_t t_ready = esp_timer_get_time();
        ESP_LOGI(TAG, "capture : buffer ready %uB %ums", (unsigned)buf_len, (unsigned)((t_ready - fr_start) / 1000));

        esp_camera_fb_return(fb);
        
        //ESP_LOGI(TAG, "capture_handler(): sending response (elapsed %ums)", (unsigned)((esp_timer_get_time() - fr_start) / 1000));
        AsyncWebServerResponse *response = request->beginResponse(200, "image/jpeg", buf, buf_len);
        response->addHeader("Content-Disposition", "inline; filename=capture.jpg");
        response->addHeader("Access-Control-Allow-Origin", "*");
        response->addHeader("Connection", "close");
        
        char ts[32];
        snprintf(ts, 32, "%lld", esp_timer_get_time() / 1000);
        response->addHeader("X-Timestamp", ts);
        
        request->send(response);
        //ESP_LOGI(TAG, "capture_handler(): request->send returned (elapsed %ums) freeHeap=%u", (unsigned)((esp_timer_get_time() - fr_start) / 1000), (unsigned)ESP.getFreeHeap());
        
        // Do not free(buf) immediately — AsyncWebServer will send it asynchronously.
        // Schedule a delayed free (2s) to allow send to complete and avoid use-after-free.
        if (buf) {
            esp_timer_handle_t once_timer;
            esp_timer_create_args_t targs = {
                .callback = [](void* arg){ free(arg); },
                .arg = buf,
                .name = "free_buf"
            };
            if (esp_timer_create(&targs, &once_timer) == ESP_OK) {
                // start in 2 seconds (2000000 microseconds)
                esp_timer_start_once(once_timer, 2000000);
            } else {
                // fallback: free immediately if timer creation failed
                free(buf);
            }
        }
        #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                int64_t fr_end = esp_timer_get_time();
                ESP_LOGI(TAG, "capture complete: JPG: %uB total %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
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
    // Instead of performing the long AVI capture in the HTTP handler, post a system event
    // that will trigger the background capture task. Return OK immediately to the web client.
    systeme_eve_t evt = { EVENT_PRISE_PHOTO, 0 };
    BaseType_t xres = xQueueSend(eventQueue, &evt, (TickType_t)(10 / portTICK_PERIOD_MS));
    if (xres != pdTRUE) {
        ESP_LOGE(TAG, "capture_handler_PHOTO(): failed to post EVENT_PRISE_PHOTO to eventQueue");
        request->send(500, "text/plain", "Failed to start PHOTO capture");
    } else {
        AsyncWebServerResponse *resp = request->beginResponse(200, "text/plain", "PHOTO capture started");
        resp->addHeader("Connection", "close");
        request->send(resp);
    }
}

void capture_photo_sd()
{

    camera_fb_t *fb = NULL;
    uint8_t *buf = NULL;
    size_t buf_len = 0;
    int buf_format = 0;  // Save format before returning fb
    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
        int64_t fr_start = esp_timer_get_time();
    #endif

        // Guard: ensure camera sensor initialized before calling fb_get
        sensor_t *s = esp_camera_sensor_get();
        if (!s) {
            ESP_LOGE(TAG, "Camera not initialized (capture_handler_SD)");
            return;
        }

    #ifdef CONFIG_LED_ILLUMINATOR_ENABLED
            ESP_LOGI(TAG, "capture_handler_SD(): enabling LED and capturing");
            enable_led(true);
            vTaskDelay(150 / portTICK_PERIOD_MS);
            fb = esp_camera_fb_get();
            enable_led(false);
        #else
            ESP_LOGI(TAG, "capture_SD(): capturing (no LED)");
            fb = esp_camera_fb_get();
        #endif

        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed: fb==NULL. freeHeap=%u psramFound=%d", (unsigned)ESP.getFreeHeap(), (int)psramFound());
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
        // Copy JPEG buffer before returning fb
        buf_len = fb->len;
        buf = (uint8_t*)malloc(buf_len);
        if (!buf) {
            esp_camera_fb_return(fb);
            ESP_LOGE(TAG, "Malloc failed for JPEG buffer copy");
            return;
        }
        memcpy(buf, fb->buf, buf_len);
    }
    else
    {
        frame2jpg(fb, 80, &buf, &buf_len);
        #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                    fb_len = buf_len;
        #endif
    }

    int64_t t_ready = esp_timer_get_time();
    ESP_LOGI(TAG, "capture_SD(): buffer ready %uB %ums freeHeap=%u", (unsigned)buf_len, (unsigned)((t_ready - fr_start) / 1000), (unsigned)ESP.getFreeHeap());

    // Save image dimensions before returning fb
    int img_width = fb->width;
    int img_height = fb->height;

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

        // Créer le répertoire /YYYY/MM (mois)
        snprintf(dir_path, sizeof(dir_path), "/%04d/%02d", year, month);
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
            // File name: C<ADDRESS>-YYMMDD-HHMMSS-<Global>-<nb><size><comp>.jpg
            int yy = year % 100;
            // Global code fixed to 'Z'
            char global_code = 'Z';


            // 2) Framesize code: derive from actual image width using size_to_code
            uint8_t size_code = 0;
            framesize_t cam_size = (framesize_t)0;
            // img_width captured earlier
            size_to_code((uint16_t)img_width, cam_size, size_code);
            Serial.printf("Size: img_width=%d, -> cam_size=%d, code=%d\n", img_width, (int)cam_size, (int)size_code);

            // 3) Compression code: normalize using code_to_compjpg
            uint8_t comp_txJpg = 0, comp_txCam = 0;
            code_to_compjpg((uint8_t)cap_jpg_comp, comp_txJpg, comp_txCam);
            Serial.printf("Compression: cap_jpg_comp=%d -> txJpg=%d, txCam=%d)\n", (int)cap_jpg_comp, (int)comp_txJpg, (int)comp_txCam);

            snprintf(file_path, sizeof(file_path), "%s/C%s-%02d%02d%02d-%02d%02d%02d-%c-1%d%d.jpg",
                     dir_path, ADDRESS, yy, month, day, current_hour, current_min, current_sec,
                     global_code, size_code, comp_code);
            
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
            ESP_LOGI(TAG, "Image saved to %s", file_path);
        } else {
            ESP_LOGE(TAG, "Failed to save image to %s", file_path);
        }
    #else
        {
            AsyncWebServerResponse *resp = request->beginResponse(500, "text/plain", "SD card not available");
            resp->addHeader("Connection", "close");
            request->send(resp);
        }
    #endif
    
    if (buf) {
        free(buf);
    }
    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
            int64_t fr_end = esp_timer_get_time();
            ESP_LOGI(TAG, "capture_SD complete: JPG: %uB total %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    #endif
    return;
}

static void capture_handler_AVI(AsyncWebServerRequest *request)
{
    // Instead of performing the long AVI capture in the HTTP handler, post a system event
    // that will trigger the background capture task. Return OK immediately to the web client.
    systeme_eve_t evt = { EVENT_PRISE_VIDEO, 0 };
    BaseType_t xres = xQueueSend(eventQueue, &evt, (TickType_t)(10 / portTICK_PERIOD_MS));
    if (xres != pdTRUE) {
        ESP_LOGE(TAG, "capture_handler_AVI(): failed to post EVENT_PRISE_VIDEO to eventQueue");
        request->send(500, "text/plain", "Failed to start AVI capture");
    } else {
        AsyncWebServerResponse *resp = request->beginResponse(200, "text/plain", "AVI capture started");
        resp->addHeader("Connection", "close");
        request->send(resp);
    }
}

// Background AVI capture implementation. Can be invoked from prise_video() which runs in the
// event handling task. This function performs the AVI capture and logs results; it does not
// interact with HTTP requests.
void capture_avi_background()
{
#ifdef SDCARD
   if (sdcard_ok)
   {
    // Non-blocking, stateful AVI capture using a one-shot timer to schedule
    // subsequent frames. If no session active, initialize a new session and
    // capture the first frame. Otherwise capture a single frame and either
    // schedule the next event or finish.


    // Ensure sensor initialized
    sensor_t *s_avi = esp_camera_sensor_get();
    if (!s_avi) {
        ESP_LOGE(TAG, "capture_avi: Camera not initialized");
        return;
    }

    // Shared session timestamp (set when starting a new session)
    unsigned long t_session_start = 0;

    // If no session active -> start session and capture first frame
    if (!avi_session.active)
    {
        lectureHeure(); // update timeinfo
        t_session_start = millis(); // session start timestamp (ms)
        char dir_path[32];
        int year = timeinfo.tm_year + 1900;
        int month = timeinfo.tm_mon + 1;
        int day = timeinfo.tm_mday;
        int hour = timeinfo.tm_hour;
        int min = timeinfo.tm_min;
        int sec = timeinfo.tm_sec;

        fs::FS &fs = SD_MMC;
        // create directory /YYYY/MM
        snprintf(dir_path, sizeof(dir_path), "/%04d/%02d", year, month);
        if (!fs.exists(dir_path))
        {
            Serial.printf("Creating directory: %s\n", dir_path);
            createDir(fs, dir_path);
        }

        // Capture first frame (with optional LED)
        camera_fb_t *fb = NULL;

        #ifdef CONFIG_LED_ILLUMINATOR_ENABLED
                enable_led(true);
                vTaskDelay(150 / portTICK_PERIOD_MS);
                fb = esp_camera_fb_get();
                enable_led(false);
        #else
                fb = esp_camera_fb_get();
        #endif

        if (!fb) {
            ESP_LOGE(TAG, "capture_avi_background: Camera capture failed (first frame)");
            return;
        }

        int width = fb->width;
        int height = fb->height;
        unsigned long t_first_capture = millis();
        ESP_LOGI(TAG, "capture_avi: first frame captured in %u ms (%d X %d)", (unsigned)(t_first_capture - t_session_start), width, height);


      /*  size_t rgb_len = (size_t)width * (size_t)height * 3;
        uint8_t *rgb_buf = (uint8_t*)malloc(rgb_len);
        if (!rgb_buf) {
            esp_camera_fb_return(fb);
            ESP_LOGE(TAG, "capture_avi_background: Memory allocation failed");
            return;
        }

        unsigned long t_conv_start = millis();
        if (!fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf)) {
            free(rgb_buf);
            esp_camera_fb_return(fb);
            ESP_LOGE(TAG, "capture_avi_background: Format conversion failed");
            return;
        }
        unsigned long t_conv_end = millis();
        ESP_LOGI(TAG, "capture_avi: image conversion took %u ms", (unsigned)(t_conv_end - t_conv_start));
        esp_camera_fb_return(fb);  */

        // Build filename
        char dir_path_small[32];
        snprintf(dir_path_small, sizeof(dir_path_small), "/%04d/%02d", year, month);
        int yy = year % 100;
        char global_code = 'Z';
        // Use the user-selected code directly as images_code (clamped 1..9)
        int images_code = nbIm_to_code(cap_nb_images);
        Serial.printf("capture_avi: cap_nb_images=%d, images_code=%d\n", cap_nb_images, images_code);

        if (images_code < 1) images_code = 1;
        if (images_code > 9) images_code = 9;
        uint8_t size_code = 0; framesize_t cam_size = (framesize_t)0;
        size_to_code((uint16_t)width, cam_size, size_code);
        uint8_t txJpg=0, txCam=0;
        uint8_t comp_code = code_to_compjpg((uint8_t)cap_jpg_comp, txJpg, txCam);

        snprintf(avi_session.file_path, sizeof(avi_session.file_path), "/sdcard%s/C%s-%02d%02d%02d-%02d%02d%02d-%c-%d%d%d.avi",
                 dir_path_small, ADDRESS, yy, month, day, hour, min, sec,
                 global_code, images_code, size_code, comp_code);

        ESP_LOGI(TAG, "capture_avi: starting AVI to %s", avi_session.file_path);

        // Open avi writer
        avi_session.avi = mjpegw_open(avi_session.file_path, (uint32_t)width, (uint32_t)height, 1, NULL);
        unsigned long t_open_end = millis();
        if (!avi_session.avi) {
            //free(rgb_buf);
            ESP_LOGE(TAG, "capture_avi: Failed to open AVI file (took %u ms)", (unsigned)(t_open_end - t_session_start));
            return;
        }
        ESP_LOGI(TAG, "capture_avi: avi_open  %u ms", (unsigned)(t_open_end - t_session_start));

        uint8_t quality = 15;
        code_to_compjpg ((uint8_t)cap_jpg_comp, quality, txCam);

        // Add first frame - if camera gives JPEG, write it directly to AVI to avoid decode/encode
        if (fb->format == PIXFORMAT_JPEG) {
            unsigned long t_add_start = millis();
            mjpegw_add_frame_jpg(avi_session.avi, fb->buf, fb->len);
            unsigned long t_add_end = millis();
            ESP_LOGI(TAG, "capture_avi: mjpegw_add_frame_jpg first took %u ms", (unsigned)(t_add_end - t_session_start));
            esp_camera_fb_return(fb);
        } else
        {
            // Fallback: convert to RGB and add frame as before
            size_t rgb_len = (size_t)width * (size_t)height * 3;
            uint8_t *rgb_buf = (uint8_t*)malloc(rgb_len);
            if (!rgb_buf) {
                esp_camera_fb_return(fb);
                ESP_LOGE(TAG, "capture_avi: Memory allocation failed");
                mjpegw_close(avi_session.avi);
                avi_session.avi = NULL;
                return;
            }
            unsigned long t_conv_start = millis();
            if (!fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf)) {
                free(rgb_buf);
                esp_camera_fb_return(fb);
                ESP_LOGE(TAG, "capture_avi: Format conversion failed");
                mjpegw_close(avi_session.avi);
                avi_session.avi = NULL;
                return;
            }
            unsigned long t_add_start = millis();
            mjpegw_add_frame(avi_session.avi, rgb_buf, quality);
            unsigned long t_add_end = millis();
            ESP_LOGI(TAG, "capture_avi: mjpegw_add_frame first took %u ms", (unsigned)(t_add_end - t_session_start));
            free(rgb_buf);
            esp_camera_fb_return(fb);
        }

        // Initialize session state
        int frames = cap_nb_images;
        avi_session.frames_total = frames;
        avi_session.frames_captured = 1;
        avi_session.frames_remaining = frames - 1;
        avi_session.width = width;
        avi_session.height = height;
        avi_session.quality = quality;
        avi_session.active = true;

        // Create timer if needed
        if (avi_session.frames_remaining > 0) {
            if (avi_session.timer == NULL) {
                avi_session.timer = xTimerCreate("AVI_T", pdMS_TO_TICKS((uint32_t)cap_interval_dsec * 100), pdFALSE, NULL, avi_timer_cb);
                if (avi_session.timer == NULL) {
                    ESP_LOGE(TAG, "capture_avi: failed to create timer");
                    // Not fatal: we will proceed but cannot schedule next frames automatically
                }
            }
            if (avi_session.timer) {
                xTimerChangePeriod(avi_session.timer, pdMS_TO_TICKS((uint32_t)cap_interval_dsec * 100), 0);
                xTimerStart(avi_session.timer, 0);
                ESP_LOGI(TAG, "capture_avi: scheduled next frame in %ums", (unsigned)(cap_interval_dsec*100));
            }
        } else {
            // No more frames required; finalize immediately
            mjpegw_close(avi_session.avi);
            avi_session.avi = NULL;
            avi_session.active = false;
            ESP_LOGI(TAG, "capture_avi: AVI saved to SD card: %s", avi_session.file_path);
        }

        return;
    }

    // Session already active: capture a single frame and either schedule next or finish
    if (avi_session.active && avi_session.avi) {
    #ifdef CONFIG_LED_ILLUMINATOR_ENABLED
            enable_led(true);
            vTaskDelay(80 / portTICK_PERIOD_MS);
    #endif
        camera_fb_t *fbi = esp_camera_fb_get();
        if (!fbi) {
            ESP_LOGE(TAG, "capture_avi: failed to get frame during session");
            // Optionally reschedule to try again
            if (avi_session.timer) xTimerStart(avi_session.timer, 0);
            return;
        }
        unsigned long t_frame_capture = millis();
        ESP_LOGI(TAG, "capture_avi: captured frame %d in %u ms since session start", avi_session.frames_captured + 1, (unsigned)(t_frame_capture - t_session_start));

        if (fbi->format == PIXFORMAT_JPEG)
        {
            unsigned long t_add_start = millis();
            mjpegw_add_frame_jpg(avi_session.avi, fbi->buf, fbi->len);
            unsigned long t_add_end = millis();
            ESP_LOGI(TAG, "capture_avi: mjpegw_add_frame_jpg for frame %d took %u ms", avi_session.frames_captured + 1, (unsigned)(t_add_end - t_add_start));
            esp_camera_fb_return(fbi);
        } else {
            uint8_t *rgb2 = (uint8_t*)malloc((size_t)avi_session.width * (size_t)avi_session.height * 3);
            if (!rgb2) {
                esp_camera_fb_return(fbi);
                ESP_LOGE(TAG, "capture_avi: malloc failed for frame");
                if (avi_session.timer) xTimerStart(avi_session.timer, 0);
                return;
            }
            unsigned long t_conv2_start = millis();
            if (!fmt2rgb888(fbi->buf, fbi->len, fbi->format, rgb2)) {
                free(rgb2);
                esp_camera_fb_return(fbi);
                ESP_LOGE(TAG, "capture_avi: fmt2rgb888 failed");
                if (avi_session.timer) xTimerStart(avi_session.timer, 0);
                return;
            }
            unsigned long t_conv2_end = millis();
            ESP_LOGI(TAG, "capture_avi: conversion for frame %d took %u ms", avi_session.frames_captured + 1, (unsigned)(t_conv2_end - t_conv2_start));
            esp_camera_fb_return(fbi);

            unsigned long t_add2_start = millis();
            mjpegw_add_frame(avi_session.avi, rgb2, avi_session.quality);
            unsigned long t_add2_end = millis();
            ESP_LOGI(TAG, "capture_avi: mjpegw_add_frame for frame %d took %u ms", avi_session.frames_captured + 1, (unsigned)(t_add2_end - t_add2_start));
            free(rgb2);
        }

        avi_session.frames_captured++;
        avi_session.frames_remaining--;

        if (avi_session.frames_remaining > 0) {
            // Schedule next capture via timer
            if (avi_session.timer) {
                xTimerChangePeriod(avi_session.timer, pdMS_TO_TICKS((uint32_t)cap_interval_dsec * 100), 0);
                xTimerStart(avi_session.timer, 0);
                ESP_LOGI(TAG, "capture_avi: captured %d/%d - scheduled next in %ums", avi_session.frames_captured, avi_session.frames_total, (unsigned)(cap_interval_dsec*100));
            } else {
                // If no timer available, fallback to immediate re-post (not ideal)
                systeme_eve_t evt = { EVENT_PRISE_VIDEO, 0 };
                xQueueSend(eventQueue, &evt, 0);
            }
        } else {
            // Done
            mjpegw_close(avi_session.avi);
            avi_session.avi = NULL;
            if (avi_session.timer) {
                xTimerDelete(avi_session.timer, 0);
                avi_session.timer = NULL;
            }
            avi_session.active = false;
            unsigned long t_end = millis();
            ESP_LOGI(TAG, "capture_avi: AVI capture complete %s (%d frames)", avi_session.file_path, avi_session.frames_captured);
            ESP_LOGI(TAG, "capture_avi: total session time %u ms", (unsigned)(t_end - t_session_start));
        }
    }
   }
   else
        ESP_LOGE(TAG, "capture_avi: SD card not ok");

#else
    ESP_LOGE(TAG, "capture_avi: SD card not available");
#endif
}

static void stream_handler(AsyncWebServerRequest *request)
{
    // Use chunked response generator to stream MJPEG continuously.
    // The generator will capture one frame every 300 ms and write the multipart part.

    #ifdef CONFIG_LED_ILLUMINATOR_ENABLED
        enable_led(true);
        isStreaming = true;
    #endif

    const char *boundary = _STREAM_BOUNDARY;
    const size_t boundary_len = strlen(boundary);

    // State for streaming without allocating a large contiguous buffer
    struct StreamState {
        size_t sent_total = 0;      // total bytes already sent
        char hdr[128];              // header buffer
        size_t hlen = 0;            // header length
        camera_fb_t *fb = NULL;     // camera frame when JPEG (owned until returned)
        uint8_t *jpg_buf = NULL;    // jpg buffer when produced by frame2jpg (owned)
        size_t jpg_len = 0;         // jpg length
        size_t frame_size = 0;      // total size = hlen + jpg_len + boundary_len
    };

    static StreamState state;

    // Generator writes parts in three segments: header, jpg payload, boundary
    std::function<size_t(uint8_t*, size_t, size_t)> stream_generator = [&](uint8_t *outBuf, size_t maxLen, size_t index) -> size_t {
        if (maxLen == 0) return 0;

        // If it's time to generate a new frame (index equals sent_total), capture
        if (index == state.sent_total) {
            // Delay between frames
            vTaskDelay(300 / portTICK_PERIOD_MS);

            // Clean previous resources if any (shouldn't normally exist)
            if (state.fb) { esp_camera_fb_return(state.fb); state.fb = NULL; }
            if (state.jpg_buf) { free(state.jpg_buf); state.jpg_buf = NULL; state.jpg_len = 0; }

            // Ensure sensor initialized
            sensor_t *s2 = esp_camera_sensor_get();
            if (!s2) {
                ESP_LOGE(TAG, "stream_generator(): Camera not initialized");
                return 0;
            }

            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                ESP_LOGE(TAG, "stream_generator(): Camera capture failed");
                return 0; // terminate stream
            }

            size_t jpg_len = 0;
            uint8_t *jpg_buf = NULL;

            if (fb->format == PIXFORMAT_JPEG) {
                jpg_len = fb->len;
                // Keep fb until we've finished sending its buffer
                state.fb = fb;
            } else {
                // Convert to JPEG (allocates jpg_buf)
                if (!frame2jpg(fb, 80, &jpg_buf, &jpg_len)) {
                    esp_camera_fb_return(fb);
                    ESP_LOGE(TAG, "stream_generator(): JPEG compression failed");
                    return 0;
                }
                // Returned the fb immediately since we have jpg_buf
                esp_camera_fb_return(fb);
                fb = NULL;
                state.jpg_buf = jpg_buf;
                state.jpg_len = jpg_len;
            }

            // Prepare header
            state.hlen = snprintf(state.hdr, sizeof(state.hdr), _STREAM_PART, (int)jpg_len, (long)esp_timer_get_time() / 1000000, (long)(esp_timer_get_time() % 1000000));
            state.frame_size = state.hlen + jpg_len + boundary_len;
        }

        // If no frame ready, end
        if (state.frame_size == 0) return 0;

        size_t offset = index - state.sent_total;
        if (offset >= state.frame_size) return 0;

        size_t remaining = state.frame_size - offset;
        size_t to_send = remaining;
        if (to_send > maxLen) to_send = maxLen;

        // Determine which segment we're in and copy accordingly
        size_t sent = 0;

        // Header segment
        if (offset < state.hlen) {
            size_t avail = state.hlen - offset;
            size_t c = (avail < to_send) ? avail : to_send;
            memcpy(outBuf + sent, state.hdr + offset, c);
            sent += c;
            offset += c;
            to_send -= c;
        }

        // JPEG payload segment
        if (to_send > 0 && offset >= state.hlen && offset < state.hlen + (state.fb ? state.fb->len : state.jpg_len)) {
            size_t jpg_offset = offset - state.hlen;
            size_t jpg_avail = (state.fb ? state.fb->len : state.jpg_len) - jpg_offset;
            size_t c = (jpg_avail < to_send) ? jpg_avail : to_send;
            if (state.fb) {
                memcpy(outBuf + sent, state.fb->buf + jpg_offset, c);
            } else {
                memcpy(outBuf + sent, state.jpg_buf + jpg_offset, c);
            }
            sent += c;
            offset += c;
            to_send -= c;
        }

        // Boundary segment
        if (to_send > 0 && offset >= state.hlen + (state.fb ? state.fb->len : state.jpg_len)) {
            size_t bound_offset = offset - state.hlen - (state.fb ? state.fb->len : state.jpg_len);
            size_t bound_avail = boundary_len - bound_offset;
            size_t c = (bound_avail < to_send) ? bound_avail : to_send;
            memcpy(outBuf + sent, boundary + bound_offset, c);
            sent += c;
            offset += c;
            to_send -= c;
        }

        // If we've completed sending the frame, advance sent_total and free resources
        if ((index - state.sent_total) + sent >= state.frame_size) {
            state.sent_total += state.frame_size;
            // free jpg_buf if used
            if (state.jpg_buf) { free(state.jpg_buf); state.jpg_buf = NULL; state.jpg_len = 0; }
            // return fb if used
            if (state.fb) { esp_camera_fb_return(state.fb); state.fb = NULL; }
            state.frame_size = 0;
            state.hlen = 0;
        }

        return sent;
    };

    // Start chunked response
    AsyncWebServerResponse *response = request->beginChunkedResponse(_STREAM_CONTENT_TYPE,
        [stream_generator](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
            return stream_generator(buffer, maxLen, index);
        }
    );
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("X-Framerate", "60");
    request->send(response);

    // When response ends, turn off LED
    #ifdef CONFIG_LED_ILLUMINATOR_ENABLED
        isStreaming = false;
        enable_led(false);
    #endif
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

    // Capture AVI (MJPEG stream packaged into an AVI file on SD card)
    server.on("/captureAVI", HTTP_GET, [](AsyncWebServerRequest *request){
        capture_handler_AVI(request);
    });

    // CORS preflight for stream (respond to OPTIONS requests)
    server.on("/stream", HTTP_OPTIONS, [](AsyncWebServerRequest *request){
        AsyncWebServerResponse *resp = request->beginResponse(200);
        resp->addHeader("Access-Control-Allow-Origin", "*");
        resp->addHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
        resp->addHeader("Access-Control-Allow-Headers", "Content-Type");
        request->send(resp);
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