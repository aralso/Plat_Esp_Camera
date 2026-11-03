#include "../include/variables.h"

// Definition of the PARAMS array (kept in a single translation unit to avoid multiple definition linker errors)
Param PARAMS[] = {
  {"LogD", 1, U8, 0, 5, 0, 0, nullptr, &log_detail, 0},             // registre 1 : détail logs
  {"reseau", 2, U8,  11, 14, 11, 0, nullptr, &mode_reseau, 0},
  {"nb_reset", 3, U16, 0, 65535, 0, 0, nullptr, &nb_reset, 0},

  // cycle and network-related registers mapped to SetReg order numbers
  {"Rap", 5, U8, 0, 12, 0, 0, nullptr, &mode_rapide, 0},           // registre 5 : cycle rapide 0 ou 12
  {"cycle", 6, U8, 1, 120, 15, 0, nullptr, &periode_cycle, 0},    // registre 4 : période du cycle (min)
  {"DelWS", 7, U8,  1, 30, 1, 0, nullptr, &DelaiWebsocket, 0},      // registre 6 : délai écoute websocket (s)
  {"Skip", 8, U8, 1, 50, 2,0, nullptr, &skip_graph, 0},             // registre 7 : skip graph

  // Parameters from requete_SetReg_appli
  {"SeBa", 9, U16, 1800, 4500, 3000, 0, nullptr, &Seuil_batt_sonde, 0},  // registre 9 : seuil batterie sonde (mV)
  {"FrBL", 10, U8, 1, 15, 7, 0, nullptr, &Nb_jours_Batt_log, 0},         // registre 10 : nb jours log batterie
  // 11:reglage date, 12:reglage heure, 13:reglage OTA
  {"Allu", 15, U8, 0, 1, 0, 0, nullptr, &pas_de_veille, 0},   // 0:veille 1:pas de mise en veille
  {"PVei", 16, U16, 15, 600, 30, 0, nullptr, &prolong_veille, 0}, 
          // registre 16 : duree allumage (s)

  // Application settings
  {"AcSt", 17, U8, 0, 1, 0, 0, nullptr, &action_stockage, 0},        // action stockage
  {"AcEn", 18, U8, 0, 1, 0, 0, nullptr, &action_envoi, 0},           // action envoi
  {"FrEn", 19, U8, 0, 15, 2, 0, nullptr, &freq_envoi, 0},        // frequence envoi (par mesure) 0:var, >1:fixe
  {"BooRap", 20, U8, 0, 3, 1, 0, nullptr, &boot_rapide, 0},          // registre 11 : boot rapide (0:lent 1:normal 2:rapide 3:très rapide(pas LogG-ota))
  {"Cpu", 21, U8, 40, 240, 240, 0, nullptr, &vit_cpu, 0},         // registre 11 : vitesse CPU (MHz)
  {"Ad_node", 22, U8, 48, 122,90 ,0, nullptr, &My_Address, 0},     // registre 22 : Adresse Node


  // Camera capture parameters (moved to high numbers to avoid collision with SetReg)
  {"cap_nb_images", 30, U8, 1, 128, 5, 0, nullptr, &cap_nb_images, 0},
  {"cap_interv", 31, U8, 1, 50, 10, 0, nullptr, &cap_interval_dsec, 0},  // intervalle entre images en 0.1sec
  {"cap_size", 32, U8, 1, 8, 4, 0, nullptr, &cap_size, 0},
  {"cap_jpg_comp", 33, U8, 1, 9, 3, 0, nullptr, &cap_jpg_comp, 0},

  {"latitude", 34, STR, 0, 0, 0, 0, "48.8461", &latitude, 16},
  {"longitude", 35, STR, 0, 0, 0, 0, "2.3469", &longitude, 16},

  {"im_x_debut", 36, U8, 0, 100, 0, 0, nullptr, &im_x_debut, 0},
  {"im_x_fin", 37, U8, 0, 100, 100, 0, nullptr, &im_x_fin, 0},
  {"im_y_debut", 38, U8, 0, 100, 0, 0, nullptr, &im_y_debut, 0},
  {"im_y_fin", 39, U8, 0, 100, 100, 0, nullptr, &im_y_fin, 0},


  // WiFi channel (SetReg_appli uses 41/42)
  {"Esp", 40, U8, 0, 1, 1, 0, nullptr, &esp_now_actif, 0},         // registre 40 : activation esp_now
  {"lWc", 41, U8, 0, 13, 0, 0, nullptr, &last_wifi_channel, 0},         // registre 41 : last_wifi_channel  (not persisted)
  {"WifiC", 42, U8, 1, 13, 1, 0, nullptr, &WIFI_CHANNEL, 0},         // registre 42 : last wifi channel valide (persisted)
  // 43:puissance wifi, 44:mode wifi

  // IPv4 addresses stored as four bytes
  {"ipAdd", 50, IP, 0, 0xFFFFFFFFu, 192, 0, nullptr, local_ip, 4},
  {"ipGat", 51, IP, 0, 0xFFFFFFFFu, 192, 0, nullptr, gateway, 4},
  {"ipSub", 52, IP, 0, 0xFFFFFFFFu, 255, 0, nullptr, subnet, 4},          // 255.255.255.0
  {"ipDNS", 53, IP, 0, 0xFFFFFFFFu, 8, 0, nullptr, primaryDNS, 4},        // 8.8.8.8
  {"ipDNS2", 54, IP, 0, 0xFFFFFFFFu, 8, 0, nullptr, secondaryDNS, 4},     // 8.8.4.4
  {"Rout", 55, STR, 0, 0, 0, 0,  "garches", nom_routeur, 16},             // nom routeur  
  {"Mdp", 56, STR, 0, 0, 0, 0, "196492380", mdp_routeur, 25},                
  {"WSOn", 57, U8, 0, 2, 1, 0, nullptr, &websocket_on, 0},                 // 0 ou 1
  {"WSock", 58, STR, 0, 0, 0, 0, "websocket", ip_websocket, 40},          // ws://webcam.hd.free.fr:8081
  {"WSId", 59, U8, 0, 9, 9, 0,nullptr, &id_websocket, 0},                 // 1, 2, 3
  {"MacGW", 61, STR, 0, 0, 0, 0,"00:00:00:00:00:00", mac_gw_str, 20},      // adresse mac gateway
  {"ipSrv", 62, IP, 0, 0xFFFFFFFFu, 192, 0, nullptr, ip_serveur, 4},       // adresse IP du serveur

  {"Test", 78, U16, 0, 60000, 100, 0, nullptr, &val_test, 0},             // registre 78 : Valeur test, pour essais)

};

// Provide number of entries for other translation units
const size_t PARAMS_COUNT = sizeof(PARAMS) / sizeof(PARAMS[0]);