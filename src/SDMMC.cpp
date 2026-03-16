
#include <Arduino.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "variables.h"

#include "FS.h"
#include "SD_MMC.h"
#include <ESPAsyncWebServer.h>


  //#include "io_extension.h"

// Default pins for ESP-S3
// Warning: ESP32-S3-WROOM-2 is using most of the default GPIOs (33-37) to interface with on-board OPI flash.
//   If the SD_MMC is initialized with default pins it will result in rebooting loop - please
//   reassign the pins elsewhere using the mentioned command `setPins`.
// Note: ESP32-S3-WROOM-1 does not have GPIO 33 and 34 broken out.
// Note: if it's ok to use default pins, you do not need to call the setPins

#ifdef ESP32_CAM
  int clk = 14;
  int cmd = 15;
  int d0 = 2;
#endif
#ifdef ESP32_S3
  int clk = 16;
  int cmd = 43;
  int d0 = 44;
#endif

extern AsyncWebServer server;


uint8_t listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return 1;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return 2;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
  return 0;
}

uint8_t createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
    return 0;
  } else {
    Serial.println("mkdir failed");
    return 1;
  }
}

uint8_t removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
    return 0;
  } else {
    Serial.println("rmdir failed");
    return 1;
  }
}

uint8_t readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return 1;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  return 0;
}

uint8_t writeFile(fs::FS &fs, const char *path, const uint8_t *buf, size_t len)
{
  Serial.printf("Writing JPEG: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return 1;
  }

  size_t written = file.write(buf, len);
  uint8_t res=2;

  if (written == len)
  {
    Serial.println("JPEG written successfully");
    res=0;
  }
  else
  {
    Serial.printf("Write failed: %u/%u bytes\n", written, len);
  }

  file.close();
  return res;
}


uint8_t appendFile(fs::FS &fs, const char *path, const uint8_t *buf, size_t len)
{
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return 1;
  }

  size_t written = file.write(buf, len);
  uint8_t res=2;

  if (written == len)
  {
    Serial.printf("Appended %u bytes\n", written);
    res = 0;
  }
  else
  {
    Serial.printf("Append failed: %u/%u bytes\n", written, len);
  }

  file.close();
  return res;
}

uint8_t renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
    return 0;
  } else {
    Serial.println("Rename failed");
    return 1;
  }
}

uint8_t deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
    return 0;
  } else {
    Serial.println("Delete failed");
    return 1;
  }
}

uint8_t testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %lu ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return 1;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %lu ms\n", 2048 * 512, end);
  file.close();
  return 0;
}

void server_routes_SDCARD()
{
  server.on("/GetSD", HTTP_GET, [](AsyncWebServerRequest *request) {
  });

  server.on("/SetSD", HTTP_GET, [](AsyncWebServerRequest *request) {
  });
}

uint8_t sd_init()
{

  /*DEV_I2C_Init();
  IO_EXTENSION_Init();
  IO_EXTENSION_Output(IO_EXTENSION_IO_2, 1);
  IO_EXTENSION_Output(IO_EXTENSION_IO_6, 1);*/

    // If you want to change the pin assignment on ESP32-S3 uncomment this block and the appropriate
    // line depending if you want to use 1-bit or 4-bit line.
    // Please note that ESP32 does not allow pin change and will always fail.
    if(! SD_MMC.setPins(clk, cmd, d0)){
        Serial.println("Pin change failed!");
        return 1;
    }
    

  if (!SD_MMC.begin("/sdcard", true)) {
      Serial.println("Card Mount Failed");
      return 2;
  }
  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD_MMC card attached");
    return 3;
  }

  Serial.print("SD_MMC Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

  listDir(SD_MMC, "/", 0);
  createDir(SD_MMC, "/mydir");
  listDir(SD_MMC, "/", 0);
  removeDir(SD_MMC, "/mydir");
  listDir(SD_MMC, "/", 2);
  //writeFile(SD_MMC, "/hello.txt", "Hello ");
  //appendFile(SD_MMC, "/hello.txt", "World!\n");
  readFile(SD_MMC, "/hello.txt");
  deleteFile(SD_MMC, "/foo.txt");
  renameFile(SD_MMC, "/hello.txt", "/foo.txt");
  readFile(SD_MMC, "/foo.txt");
  testFileIO(SD_MMC, "/test.txt");
  Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));

  return 0;
}