#ifndef SDMMC_H
#define SDMMC_H

// Header pour la Carte SD : SDMMC
#include <Arduino.h>
#include "FS.h"
#include "SD_MMC.h"

uint8_t sd_init();
uint8_t listDir(fs::FS &fs, const char *dirname, uint8_t levels);
uint8_t createDir(fs::FS &fs, const char *path);
uint8_t removeDir(fs::FS &fs, const char *path);
uint8_t readFile(fs::FS &fs, const char *path);
uint8_t writeFile(fs::FS &fs, const char *path, const uint8_t *buf, size_t len);
uint8_t appendFile(fs::FS &fs, const char *path, const uint8_t *buf, size_t len);
uint8_t renameFile(fs::FS &fs, const char *path1, const char *path2);
uint8_t deleteFile(fs::FS &fs, const char *path);
uint8_t server_routes_SDCARD();


#endif
