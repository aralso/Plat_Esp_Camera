#ifndef cam_pin
#define cam_pin

/*
  camera_pins.h - Pin definitions for some common ESP-CAM modules

  Part of the WS-Platformio project -

 *   Pin definitions for some common ESP-CAM modules
 *
 *   Select the module to use in myconfig.h
 *   Defaults to AI-THINKER CAM module
 *
 */

#include "variables.h"

#ifdef ESP32_v1
  #define CAMERA_MODEL_AI_THINKER
#endif
#ifdef ESP32_S3
  #define CAMERA_MODEL_ESP32_S3_CAM
#endif

 
#if defined(CAMERA_MODEL_AI_THINKER)
  //
  // AI Thinker
  // https://github.com/SeeedDocument/forum_doc/raw/master/reg/ESP32_CAM_V1.6.pdf
  //
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
  #define LED_PIN           33 // Status led
  #define LED_ON           LOW // - Pin is inverted.
  #define LED_OFF         HIGH //
  #define LAMP_PIN           4 // LED FloodLamp.

    /* --------------- SD CARD CFG  ---------------*/
  #define ENABLE_SD_CARD              true     ///< Enable SD card function
  #define SD_PIN_CLK                  14      ///< GPIO pin for SD card clock
  #define SD_PIN_CMD                  15       ///< GPIO pin for SD card command
  #define SD_PIN_DATA0                2       ///< GPIO pin for SD card data 0

#elif defined(CAMERA_MODEL_ESP32_S3_CAM)

  /* --------------- CAMERA CFG -------------------*/
  #define PWDN_GPIO_NUM               -1      ///< Power down control pin
  #define RESET_GPIO_NUM              -1      ///< Reset control pin
  #define XCLK_GPIO_NUM                15     ///< External clock pin
  #define SIOD_GPIO_NUM               4       ///< SCCB: SI/O data pin
  #define SIOC_GPIO_NUM               5       ///< SCCB: SI/O control pin
  #define Y9_GPIO_NUM                 16      ///< SCCB: Y9 pin
  #define Y8_GPIO_NUM                 17      ///< SCCB: Y8 pin
  #define Y7_GPIO_NUM                 18      ///< SCCB: Y7 pin
  #define Y6_GPIO_NUM                 12      ///< SCCB: Y6 pin
  #define Y5_GPIO_NUM                 10      ///< SCCB: Y5 pin
  #define Y4_GPIO_NUM                 8       ///< SCCB: Y4 pin
  #define Y3_GPIO_NUM                 9       ///< SCCB: Y3 pin
  #define Y2_GPIO_NUM                 11      ///< SCCB: Y2 pin
  #define VSYNC_GPIO_NUM              6       ///< Vertical sync pin
  #define HREF_GPIO_NUM               7       ///< Line sync pin
  #define PCLK_GPIO_NUM               13      ///< Pixel clock pin

  /* ------------------ MCU CFG  ------------------*/
  #define BOARD_NAME                  F("ESP32-S3-CAM") ///< Board name
  #define ENABLE_BROWN_OUT_DETECTION  false   ///< Enable brown out detection
  #define ENABLE_PSRAM                true    ///< Enable PSRAM   

  /* --------------- OTA UPDATE CFG  --------------*/
  #define OTA_UPDATE_FW_FILE          PSTR("esp32-s3-cam.bin") ///< OTA update firmware file name
  #define FW_STATUS_LED_PIN           2        ///< GPIO pin for status FW update LED
  #define FW_STATUS_LED_LEVEL_ON      HIGH     ///< GPIO pin level for status LED ON

  /* --------------- FLASH LED CFG  ---------------*/
  #define ENABLE_CAMERA_FLASH         true     ///< Enable camera flash function
  #define CAMERA_FLASH_DIGITAL_CTRL   true     ///< Enable camera flash digital control
  #define CAMERA_FLASH_PWM_CTRL       false    ///< Enable camera flash PWM control
  #define CAMERA_FLASH_NEOPIXEL       true     ///< Enable camera flash NeoPixel control
  #define FLASH_GPIO_NUM              47       ///< Flash control pin.
  #define FLASH_NEOPIXEL_LED_PIN      48       ///< External flash control pin. RGB LED NeoPixel
  #define FLASH_OFF_STATUS            LOW      ///< value for turn off flash
  #define FLASH_ON_STATUS             HIGH     ///< value for turn on flash
  //#define FLASH_PWM_FREQ              2000    ///< frequency of pwm [240MHz / (100 prescale * pwm cycles)] = frequency
  //#define FLASH_PWM_CHANNEL           0       ///< channel 0
  //#define FLASH_PWM_RESOLUTION        8       ///< range 1-20bit. 8bit = 0-255 range

  /* --------------- SD CARD CFG  ---------------*/
  #define ENABLE_SD_CARD              true     ///< Enable SD card function
  #define SD_PIN_CLK                  39       ///< GPIO pin for SD card clock
  #define SD_PIN_CMD                  38       ///< GPIO pin for SD card command
  #define SD_PIN_DATA0                40       ///< GPIO pin for SD card data 0

  /* ---------- RESET CFG CONFIGURATION  ----------*/
  #define CFG_RESET_PIN               14       ///< GPIO 16 is for reset CFG to default
  #define CFG_RESET_LED_PIN           2        ///< GPIO for indication of reset CFG
  #define CFG_RESET_LED_LEVEL_ON      HIGH     ///< GPIO pin level for status LED ON

  /* -------------- STATUS LED CFG ----------------*/
  #define STATUS_LED_ENABLE           true     ///< enable/disable status LED
  #define STATUS_LED_GPIO_NUM         2        ///< GPIO pin for status LED
  #define STATUS_LED_OFF_PIN_LEVEL    HIGH     ///< GPIO pin level for status LED ON

  /* -------------- DHT SENSOR CFG ----------------*/
  #define DHT_SENSOR_ENABLE           true     ///< enable/disable DHT sensor
  #define DHT_SENSOR_PIN              20       ///< GPIO pin for DHT sensor


#elif defined(CAMERA_MODEL_WROVER_KIT)
  //
  // ESP WROVER
  // https://dl.espressif.com/dl/schematics/ESP-WROVER-KIT_SCH-2.pdf
  //
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22
  #define LED_PIN           2 // A status led on the RGB; could also use pin 0 or 4
  #define LED_ON         HIGH //
  #define LED_OFF         LOW //
  // #define LAMP_PIN          x // No LED FloodLamp.

#elif defined(CAMERA_MODEL_ESP_EYE)
  //
  // ESP-EYE
  // https://twitter.com/esp32net/status/1085488403460882437
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM     4
  #define SIOD_GPIO_NUM    18
  #define SIOC_GPIO_NUM    23
  #define Y9_GPIO_NUM      36
  #define Y8_GPIO_NUM      37
  #define Y7_GPIO_NUM      38
  #define Y6_GPIO_NUM      39
  #define Y5_GPIO_NUM      35
  #define Y4_GPIO_NUM      14
  #define Y3_GPIO_NUM      13
  #define Y2_GPIO_NUM      34
  #define VSYNC_GPIO_NUM    5
  #define HREF_GPIO_NUM    27
  #define PCLK_GPIO_NUM    25
  #define LED_PIN          21 // Status led
  #define LED_ON         HIGH //
  #define LED_OFF         LOW //
  // #define LAMP_PIN          x // No LED FloodLamp.

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  //
  // ESP32 M5STACK
  //
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21
  // M5 Stack status/illumination LED details unknown/unclear
  // #define LED_PIN            x // Status led
  // #define LED_ON          HIGH //
  // #define LED_OFF          LOW //
  // #define LAMP_PIN          x  // LED FloodLamp.

#elif defined(CAMERA_MODEL_M5STACK_V2_PSRAM)
  //
  // ESP32 M5STACK V2
  //
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     22
  #define SIOC_GPIO_NUM     23
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21
  // M5 Stack status/illumination LED details unknown/unclear
  // #define LED_PIN            x // Status led
  // #define LED_ON          HIGH //
  // #define LED_OFF          LOW //
  // #define LAMP_PIN          x  // LED FloodLamp.

#elif defined(CAMERA_MODEL_M5STACK_WIDE)
  //
  // ESP32 M5STACK WIDE
  //
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     22
  #define SIOC_GPIO_NUM     23
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21
  // M5 Stack status/illumination LED details unknown/unclear
  // #define LED_PIN            x // Status led
  // #define LED_ON          HIGH //
  // #define LED_OFF          LOW //
  // #define LAMP_PIN          x  // LED FloodLamp.

#elif defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  //
  // Common M5 Stack without PSRAM
  //
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21
  // Note NO PSRAM,; so maximum working resolution is XGA 1024×768
  // M5 Stack status/illumination LED details unknown/unclear
  // #define LED_PIN            x // Status led
  // #define LED_ON          HIGH //
  // #define LED_OFF          LOW //
  // #define LAMP_PIN          x  // LED FloodLamp.

#elif defined(CAMERA_MODEL_TTGO_T_JOURNAL)
  //
  // LilyGO TTGO T-Journal ESP32; with OLED! but not used here.. :-(
  #define PWDN_GPIO_NUM      0
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21
  // TTGO T Journal status/illumination LED details unknown/unclear
  // #define LED_PIN           33 // Status led
  // #define LED_ON           LOW // - Pin is inverted.
  // #define LED_OFF         HIGH //
  // #define LAMP_PIN           4 // LED FloodLamp.

#elif defined(CAMERA_MODEL_ARDUCAM_ESP32S_UNO)
  // Pins from user @rdragonrydr
  // https://github.com/ArduCAM/ArduCAM_ESP32S_UNO/
  // Based on AI-THINKER definitions
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
  #define LED_PIN            2 // Status led
  #define LED_ON          HIGH // - Pin is not inverted.
  #define LED_OFF          LOW //
  //#define LAMP_PIN           x // No LED FloodLamp.

#else
  // Well.
  // that went badly...
  #error "Camera model not selected, did you forget to uncomment it in myconfig?"
#endif

#endif