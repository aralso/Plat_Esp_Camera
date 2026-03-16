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

/*Peux-tu créer une page html avec du javascript, mais avec une page de taille plutot petite,
qui fasse les fonctionnalités suivantes : 
- lecture de la carte SD , au niveau de la racine, et présentation dans la page
de ce répertoire racine, avec les repértoires et le fichiers du répertoire racine.
Trie la liste selon la date descendante.
Présente les champs suivants pour chaque ligne :
  -icone différente pour les répertoires et pour les fichiers
  -nom du fichier ou du repertoire
  -date de modification du fichier/repertoire
  -taille du fichier
  -plusieurs icones : ouvrir, télécharger, supprimer, renommer, déplacer
Ajoute 2 boutons/icones en haut du fichier :
1 pour créer un répertoire, 1 pour mettre à jour la page en présentant le niveau d'arboresence supérieur

Si on clique sur ouvrir ou sur le nom du fichier/repertoire, si c'est un fichier : cela ouvre le fichier dans l'explorateur 
si c'est un repertoire, cela met à jour la page avec le contenu du nouveau répertoire
les autres icones font ce qui correspond à leur nom

/set : 
création nouveau repertoire X
supprimer fichier/rep x
renommer fichier/rep x en y
déplacer fichier/rep x vers y

/get :
lecture repertoire x
ouvrir fichier/repertoire x
télécharger fichier x

complète aussi le fichier SDMMC.cpp, qui execute les fonctions dans l'esp32 avec les contraintes suivantes : 
Pour les lectures, utilise la requete suivante :
 /getSD?action=xx&path=xx&fs=xx  action=1 pour lire un rep, action=2 pour ouvrir fichier/rep, action=3 pour télécharger fichier
  server.on("/GetSD", HTTP_GET, [](AsyncWebServerRequest *request)

Pour les écritures, utilise la requete suivante : 
/setSD?action=xx&path=xx&fs=xx&out=xx avec
action=1 (création nouv rep), action=2 (supp fichier/rep), action=3 (renommage), action=4 (déplacer)

Ecrit ces 2 requetes, qui sont citées dans la fonction server_routes_SDCARD(); utilise les fonctions existantes du fichier sdmmc.cpp
*/
  #endif
