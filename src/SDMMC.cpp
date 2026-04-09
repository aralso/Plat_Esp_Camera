
#include <Arduino.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

#include "variables.h"

#include "FS.h"
#include "SD_MMC.h"
#include <ESPAsyncWebServer.h>

#include "camera_pins.h"

// HTML page for SD explorer
const char sd_explorer_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Explorateur SD</title>
    <style>
        body { font-family: Arial, sans-serif; background: #181818; color: #EFEFEF; margin: 0; padding: 10px; }
        .header { display: flex; gap: 10px; margin-bottom: 10px; }
        .header button { padding: 5px 10px; background: #363636; color: #EFEFEF; border: 1px solid #555; cursor: pointer; }
        .header button:hover { background: #555; }
        table { width: 100%; border-collapse: collapse; }
        th, td { border: 1px solid #555; padding: 5px; text-align: left; }
        th { background: #363636; }
        .icon { width: 20px; height: 20px; display: inline-block; margin-right: 5px; }
        .dir-icon { background: url('data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24"><path fill="%23FFD700" d="M10 4H4c-1.1 0-1.99.9-1.99 2L2 18c0 1.1.89 2 2 2h16c1.1 0 2-.9 2-2V8c0-1.1-.9-2-2-2h-8l-2-2z"/></svg>') no-repeat center; }
        .file-icon { background: url('data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24"><path fill="%23FFFFFF" d="M14,2H6A2,2 0 0,0 4,4V20A2,2 0 0,0 6,22H18A2,2 0 0,0 20,20V8L14,2M18,20H6V4H13V9H18V20Z"/></svg>') no-repeat center; }
        .action-icon { width: 16px; height: 16px; cursor: pointer; margin: 0 2px; }
        .open-icon { background: url('data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24"><path fill="%2300FF00" d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z"/></svg>') no-repeat center; }
        .download-icon { background: url('data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24"><path fill="%2300FF00" d="M19 9h-4V3H9v6H5l7 7 7-7zM5 18v2h14v-2H5z"/></svg>') no-repeat center; }
        .delete-icon { background: url('data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24"><path fill="%23FF0000" d="M6 19c0 1.1.9 2 2 2h8c1.1 0 2-.9 2-2V7H6v12zM19 4h-3.5l-1-1h-5l-1 1H5v2h14V4z"/></svg>') no-repeat center; }
        .rename-icon { background: url('data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24"><path fill="%23FFFF00" d="M3 17.25V21h3.75L17.81 9.94l-3.75-3.75L3 17.25zM20.71 7.04c.39-.39.39-1.02 0-1.41l-2.34-2.34c-.39-.39-1.02-.39-1.41 0l-1.83 1.83 3.75 3.75 1.83-1.83z"/></svg>') no-repeat center; }
        .move-icon { background: url('data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24"><path fill="%2300FFFF" d="M16 17.01V10h-2v7.01h-3L15 21l4-3.99h-3zM9 3L5 6.99h3V14h2V6.99h3L9 3z"/></svg>') no-repeat center; }
        .clickable { cursor: pointer; color: #00FFFF; }
    </style>
</head>
<body>
    <div class="header">
        <button id="createDir">Créer Répertoire</button>
        <button id="goUp">Niveau Supérieur</button>
    </div>
    <table id="fileTable">
        <thead>
            <tr>
                <th>Type</th>
                <th>Nom</th>
                <th>Date Modification</th>
                <th>Taille</th>
                <th>Actions</th>
            </tr>
        </thead>
        <tbody id="fileList">
        </tbody>
    </table>

    <script>
        let currentPath = '/';

        function loadDirectory(path) {
            currentPath = path;
            fetch(`/getSD?action=1&path=${encodeURIComponent(path)}&fs=SD_MMC`)
                .then(response => response.json())
                .then(data => {
                    const tbody = document.getElementById('fileList');
                    tbody.innerHTML = '';
                    data.forEach(item => {
                        const row = document.createElement('tr');
                        const iconClass = item.isDir ? 'dir-icon' : 'file-icon';
                        const size = item.isDir ? '-' : formatSize(item.size);
                        const date = new Date(item.mtime * 1000).toLocaleString();
                        row.innerHTML = `
                            <td><div class="icon ${iconClass}"></div></td>
                            <td class="clickable" onclick="openItem('${item.name}', ${item.isDir})">${item.name}</td>
                            <td>${date}</td>
                            <td>${size}</td>
                            <td>
                                <div class="icon open-icon action-icon" onclick="openItem('${item.name}', ${item.isDir})"></div>
                                ${!item.isDir ? '<div class="icon download-icon action-icon" onclick="downloadFile(\'' + item.name + '\')"></div>' : ''}
                                <div class="icon delete-icon action-icon" onclick="deleteItem('${item.name}', ${item.isDir})"></div>
                                <div class="icon rename-icon action-icon" onclick="renameItem('${item.name}')"></div>
                                <div class="icon move-icon action-icon" onclick="moveItem('${item.name}')"></div>
                            </td>
                        `;
                        tbody.appendChild(row);
                    });
                })
                .catch(error => console.error('Error loading directory:', error));
        }

        function joinPath(dir, name) {
            const cleanDir = dir.endsWith('/') && dir.length > 1 ? dir.slice(0, -1) : dir;
            const cleanName = name.startsWith('/') ? name.slice(1) : name;
            if (cleanDir === '/') return `/${cleanName}`;
            return `${cleanDir}/${cleanName}`;
        }

        function openItem(name, isDir) {
            const fullPath = joinPath(currentPath, name);
            if (isDir) {
                loadDirectory(fullPath);
            } else {
                window.open(`/getSD?action=2&path=${encodeURIComponent(fullPath)}&fs=SD_MMC`);
            }
        }

        function downloadFile(name) {
            const fullPath = joinPath(currentPath, name);
            window.open(`/getSD?action=3&path=${encodeURIComponent(fullPath)}&fs=SD_MMC`);
        }

        function deleteItem(name, isDir) {
            const fullPath = joinPath(currentPath, name);
            if (confirm(`Supprimer ${isDir ? 'le répertoire' : 'le fichier'} ${name} ?`)) {
                fetch(`/setSD?action=2&path=${encodeURIComponent(fullPath)}&fs=SD_MMC&out=`)
                    .then(response => {
                        if (response.ok) {
                            loadDirectory(currentPath);
                        } else {
                            alert('Erreur lors de la suppression');
                        }
                    });
            }
        }

        function renameItem(name) {
            const newName = prompt('Nouveau nom:', name);
            if (newName && newName !== name) {
                const oldPath = joinPath(currentPath, name);
                const newPath = joinPath(currentPath, newName);
                fetch(`/setSD?action=3&path=${encodeURIComponent(oldPath)}&fs=SD_MMC&out=${encodeURIComponent(newPath)}`)
                    .then(response => {
                        if (response.ok) {
                            loadDirectory(currentPath);
                        } else {
                            alert('Erreur lors du renommage');
                        }
                    });
            }
        }

        function moveItem(name) {
            const dest = prompt('Destination (chemin complet):', currentPath);
            if (dest) {
                const srcPath = joinPath(currentPath, name);
                const destPath = joinPath(dest, name);
                fetch(`/setSD?action=4&path=${encodeURIComponent(srcPath)}&fs=SD_MMC&out=${encodeURIComponent(destPath)}`)
                    .then(response => {
                        if (response.ok) {
                            loadDirectory(currentPath);
                        } else {
                            alert('Erreur lors du déplacement');
                        }
                    });
            }
        }

        document.getElementById('createDir').onclick = () => {
            const dirName = prompt('Nom du nouveau répertoire:');
            if (dirName) {
                const path = joinPath(currentPath, dirName);
                fetch(`/setSD?action=1&path=${encodeURIComponent(path)}&fs=SD_MMC&out=`)
                    .then(response => {
                        if (response.ok) {
                            loadDirectory(currentPath);
                        } else {
                            alert('Erreur lors de la création');
                        }
                    });
            }
        };

        document.getElementById('goUp').onclick = () => {
            if (currentPath !== '/') {
                const parts = currentPath.split('/');
                parts.pop();
                const upPath = parts.length === 1 ? '/' : parts.join('/');
                loadDirectory(upPath);
            }
        };

        function formatSize(bytes) {
            if (bytes === 0) return '0 B';
            const k = 1024;
            const sizes = ['B', 'KB', 'MB', 'GB'];
            const i = Math.floor(Math.log(bytes) / Math.log(k));
            return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
        }

        // Load root on start
        loadDirectory('/');
    </script>
</body>
</html>
)rawliteral";


  //#include "io_extension.h"

// Default pins for ESP-S3
// Warning: ESP32-S3-WROOM-2 is using most of the default GPIOs (33-37) to interface with on-board OPI flash.
//   If the SD_MMC is initialized with default pins it will result in rebooting loop - please
//   reassign the pins elsewhere using the mentioned command `setPins`.
// Note: ESP32-S3-WROOM-1 does not have GPIO 33 and 34 broken out.
// Note: if it's ok to use default pins, you do not need to call the setPins

/*#ifdef ESP32_v1
  int clk = 14;
  int cmd = 15;
  int d0 = 2;
#endif
#ifdef ESP32_S3
  int clk = 47; //16;
  int cmd = 21; //43;
  int d0 = 48; //44;
#endif*/

extern AsyncWebServer server;

String getContentType(const String& filename) {

  if (filename.endsWith(".html") || filename.endsWith(".htm")) return "text/html";
  if (filename.endsWith(".css")) return "text/css";
  if (filename.endsWith(".js")) return "application/javascript";

  if (filename.endsWith(".png")) return "image/png";
  if (filename.endsWith(".gif")) return "image/gif";
  if (filename.endsWith(".jpg") || filename.endsWith(".jpeg")) return "image/jpeg";
  if (filename.endsWith(".svg")) return "image/svg+xml";
  if (filename.endsWith(".ico")) return "image/x-icon";

  if (filename.endsWith(".mp4")) return "video/mp4";
  if (filename.endsWith(".mpeg") || filename.endsWith(".mpg")) return "video/mpeg";

  if (filename.endsWith(".mp3")) return "audio/mpeg";
  if (filename.endsWith(".wav")) return "audio/wav";

  if (filename.endsWith(".pdf")) return "application/pdf";
  if (filename.endsWith(".json")) return "application/json";
  if (filename.endsWith(".txt")) return "text/plain";

  return "application/octet-stream"; // fallback
}

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

bool createDir(fs::FS &fs, const char *path) {
    String current = "";
    String fullPath = String(path);

    for (int i = 0; i < fullPath.length(); i++) {
        char c = fullPath[i];

        // éviter double slash
        if (c == '/' && current.endsWith("/")) continue;

        current += c;

        // créer uniquement si ce n'est pas le dernier caractère
        if ((c == '/' && current.length() > 1) || i == fullPath.length() - 1) {

            // supprimer slash final temporairement
            String dir = current;
            if (dir.endsWith("/") && dir.length() > 1) {
                dir.remove(dir.length() - 1);
            }

            if (!fs.exists(dir)) {
                if (!fs.mkdir(dir)) {
                    Serial.println("Erreur mkdir: " + dir);
                    return false;
                }
            }
        }
    }
    return true;
}

/*uint8_t createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
    return 0;
  } else {
    Serial.println("mkdir failed");
    return 1;
  }
}*/

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
  struct Item {
    String name;
    bool isDir;
    time_t mtime;
    size_t size;
  };

  server.on("/sd", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", sd_explorer_html);
  });

  server.on("/getSD", HTTP_GET, [](AsyncWebServerRequest *request) {
    int action = request->getParam("action")->value().toInt();
    String path = request->getParam("path")->value();
    String fs_str = request->getParam("fs")->value();
    fs::FS &fs = SD_MMC; // Assuming fs is SD_MMC

    // Normalize path for SD_MMC: ESP32 mounts SD_MMC at "/sdcard" by default.
    /*if (fs_str == "SD_MMC") {
      const String mountPoint = "/sdcard";
      if (path == "/") {
        path = mountPoint;
      } else if (!path.startsWith(mountPoint)) {
        path = mountPoint + path;
      }
    }*/

    if (action == 1)
    { // Read directory
      File root = fs.open(path);
      if (!root || !root.isDirectory()) {
        request->send(404, "text/plain", "Directory not found");
        return;
      }

      auto normalizeName = [&](const String &fullName) {
        String name = fullName;
        if (path != "/" && name.startsWith(path)) {
          name = name.substring(path.length());
        }
        if (name.startsWith("/")) {
          name = name.substring(1);
        }
        return name;
      };

      String json = "[";
      File file = root.openNextFile();
      std::vector<Item> items;
      while (file) {
        Item item;
        item.name = normalizeName(String(file.name()));
        item.isDir = file.isDirectory();
        item.mtime = file.getLastWrite();
        item.size = item.isDir ? 0 : file.size();
        items.push_back(item);
        file = root.openNextFile();
      }
      // Sort by mtime descending
      std::sort(items.begin(), items.end(), [](const Item &a, const Item &b) {
        return a.mtime > b.mtime;
      });
      for (size_t i = 0; i < items.size(); i++) {
        if (i > 0) json += ",";
        json += "{\"name\":\"" + items[i].name + "\",\"isDir\":" + (items[i].isDir ? "true" : "false") + ",\"mtime\":" + String(items[i].mtime) + ",\"size\":" + String(items[i].size) + "}";
      }
      json += "]";
      request->send(200, "application/json", json);
    }
    else if (action == 2)
    { // Open file/dir
      File file = fs.open(path);
      if (!file) {
        request->send(404, "text/plain", "File not found");
        return;
      }
      if (file.isDirectory()) {
        // For dir, return JSON like action=1
        auto normalizeName = [&](const String &fullName) {
          String name = fullName;
          if (path != "/" && name.startsWith(path)) {
            name = name.substring(path.length());
          }
          if (name.startsWith("/")) {
            name = name.substring(1);
          }
          return name;
        };

        String json = "[";
        File subfile = file.openNextFile();
        std::vector<Item> items;
        while (subfile) {
          Item item;
          item.name = normalizeName(String(subfile.name()));
          item.isDir = subfile.isDirectory();
          item.mtime = subfile.getLastWrite();
          item.size = item.isDir ? 0 : subfile.size();
          items.push_back(item);
          subfile = subfile.openNextFile();
        }
        std::sort(items.begin(), items.end(), [](const Item &a, const Item &b) {
          return a.mtime > b.mtime;
        });
        for (size_t i = 0; i < items.size(); i++) {
          if (i > 0) json += ",";
          json += "{\"name\":\"" + items[i].name + "\",\"isDir\":" + (items[i].isDir ? "true" : "false") + ",\"mtime\":" + String(items[i].mtime) + ",\"size\":" + String(items[i].size) + "}";
        }
        json += "]";
        request->send(200, "application/json", json);
      }
      else
      {
        // For file, send file content (open in browser)
        String filename = path;
        int lastSlash = filename.lastIndexOf('/');
        if (lastSlash >= 0) {
          filename = filename.substring(lastSlash + 1);
        }
        String contentType = getContentType(filename);
        auto response = request->beginResponse(fs, path, contentType);
        response->addHeader("Content-Disposition", "inline; filename=\"" + filename + "\"");
        request->send(response);
      }
    }
    else if (action == 3)
    { // Download file
      String filename = path;
      int lastSlash = filename.lastIndexOf('/');
      if (lastSlash >= 0) {
        filename = filename.substring(lastSlash + 1);
      }
      auto response = request->beginResponse(fs, path, "application/octet-stream");
      response->addHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
      request->send(response);
    } else
    {
      request->send(400, "text/plain", "Invalid action");
    }
  });

  server.on("/setSD", HTTP_GET, [](AsyncWebServerRequest *request) {
    int action = request->getParam("action")->value().toInt();
    String path = request->getParam("path")->value();
    String fs_str = request->getParam("fs")->value();
    String out = request->getParam("out")->value();
    fs::FS &fs = SD_MMC;

    // Normalize path for SD_MMC mount point
    /*if (fs_str == "SD_MMC") {
      const String mountPoint = "/sdcard";
      if (path == "/") {
        path = mountPoint;
      } else if (!path.startsWith(mountPoint)) {
        path = mountPoint + path;
      }
      if (!out.isEmpty() && out == "/") {
        out = mountPoint;
      } else if (!out.isEmpty() && !out.startsWith(mountPoint)) {
        out = mountPoint + out;
      }
    }*/

    uint8_t result = 1;
    if (action == 1) { // Create dir
      result = createDir(fs, path.c_str());
    } else if (action == 2) { // Delete file/dir
      File file = fs.open(path);
      if (file.isDirectory()) {
        result = removeDir(fs, path.c_str());
      } else {
        result = deleteFile(fs, path.c_str());
      }
    } else if (action == 3) { // Rename
      result = renameFile(fs, path.c_str(), out.c_str());
    } else if (action == 4) { // Move (rename with path)
      result = renameFile(fs, path.c_str(), out.c_str());
    } else {
      request->send(400, "text/plain", "Invalid action");
      return;
    }
    if (result == 0) {
      request->send(200, "text/plain", "OK");
    } else {
      request->send(500, "text/plain", "Error");
    }
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
    if(! SD_MMC.setPins(SD_PIN_CLK, SD_PIN_CMD, SD_PIN_DATA0)){  // 14, 15, 2)) { //
        Serial.println("Pin change failed!");
        return 1;
    }
    delay(100);
    
    Serial.println("AAA");
    delay(1000);


  if (!SD_MMC.begin("/sdcard", true)) {
      Serial.println("Card Mount Failed");
      return 2;
  }
    Serial.println("BBB");
    delay(1000);

  uint8_t cardType = SD_MMC.cardType();

    Serial.println("CCC");
    delay(1000);

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
    Serial.println("SDHC");  // carte sd
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

  const char* mountPoint = "/";
  listDir(SD_MMC, mountPoint, 0);
  //createDir(SD_MMC, "/mydir");
  //mountPoint = "/mydir";
  //listDir(SD_MMC, mountPoint, 1);
  //removeDir(SD_MMC, "/mydir");
  //listDir(SD_MMC, "/", 2);
  //writeFile(SD_MMC, "/hello.txt", "Hello ");
  //appendFile(SD_MMC, "/hello.txt", "World!\n");
  //readFile(SD_MMC, "/hello.txt");
  //deleteFile(SD_MMC, "/foo.txt");
  //renameFile(SD_MMC, "/hello.txt", "/sdcard/foo.txt");
  //readFile(SD_MMC, "/foo.txt");
  //testFileIO(SD_MMC, "/test.txt");
  Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));

  return 0;
}