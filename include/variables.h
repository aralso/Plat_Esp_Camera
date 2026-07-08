#ifndef VARIABLES_H

#define VARIABLES_H

#include <stddef.h>  // for size_t

#ifdef __cplusplus
  #include <Arduino.h>  // for IPAddress, String types
  // Forward-declare C++ LPC types/functions so C files including this header don't
  // try to parse C++-only declarations or include C++ headers like <cassert>.
  struct lpc_settings_t;
  int encode_lpc(const lpc_settings_t &settings);
#else
  // C-mode: provide opaque declarations for Arduino C++ types so C files can
  // reference extern variables without including Arduino headers.
  typedef struct IPAddress IPAddress;
  typedef struct String String;
#endif


// variables externes



// Hardware
//#define MODE_WT32  // WT32-Eth01 sinon ESP32-CAM ou DOIT ESP32 Devkit V1

//#define DEBUG  // mode station, pas de websocket, pas de sécurite, emulation valeurs STM32
//#define ESP32_v1    // DOIT ESP32 DEVKIt V1


//#define ESP32_v1  // esp32cam
//#define ESP32_uPesy  // uPesy_vroom
#define ESP32_S3
//#define ESP32_Fire2


//#define Temp_int_HDC1080  // Capteur I2C HDC1080
#define MODE_Wifi  // Wifi sinon Ethernet
//#define Sans_securite
#define Sans_websocket
//#define WatchDog
#define Sans_EspNow
//#define ESP_VEILLE

#define SDCARD
#define CAMERA
//#define Temp_int_DHT22

// Device address identifier used in filenames (single letter or short string)
#ifndef ADDRESS
#define ADDRESS "A"
#endif

//#define Temp_int_DS18B20

// Réseau
//#define NO_RESEAU

//#define Wifi_AP    // AP sinon STA

//#define STM32  //incompatible du modbus, sauf à changer les pin
// #define OTA


// Definir le canal WIFI ici (doit correspondre au routeur pour l'esp_remote)
// ⚠️ IMPORTANT : Ce canal DOIT correspondre au canal de votre routeur WiFi
// "garches" Pour le trouver : regardez les logs de l'esp_remote au démarrage


typedef struct {
  uint8_t type;  // 1: Temperature, 2: Batterie
  float value;
} Message_EspNow;

// structure des paramètres 
typedef enum ParamType {
  U8,
  U16,
  IP,
  STR,
  U32
} ParamType;

typedef struct Param {
  const char* key;
  uint8_t order;
  ParamType type;
 
  uint32_t min16;   // numeric lower bound (used for U8/U16/U32)
  uint32_t max16;   // numeric upper bound
 
  uint32_t def_u16; // default numeric value (fits U8/U16/U32)
  uint8_t rtc_valid;  // 0: not valid, 1: valid
  const char* def_str;
  void* var;
  uint8_t size;      // taille du buffer (0 pour U8/U16)
} Param;

// Forward declarations for variables used in PARAMS
extern uint8_t mode_reseau;
extern uint16_t nb_reset;
extern RTC_DATA_ATTR uint8_t periode_cycle;
extern RTC_DATA_ATTR uint8_t mode_rapide;
extern uint8_t log_detail;
extern uint8_t DelaiWebsocket;
extern RTC_DATA_ATTR uint8_t skip_graph;
extern uint16_t Seuil_batt_sonde;
extern RTC_DATA_ATTR uint8_t Nb_jours_Batt_log;
extern RTC_DATA_ATTR uint16_t prolong_veille;
extern RTC_DATA_ATTR uint8_t action_stockage;
extern RTC_DATA_ATTR uint8_t action_envoi;
extern char nom_routeur[];
extern char mdp_routeur[];
extern uint8_t websocket_on;
extern char ip_websocket[];
extern uint8_t id_websocket;
extern uint8_t WIFI_CHANNEL;
extern RTC_DATA_ATTR uint8_t last_wifi_channel;
extern IPAddress local_ip;
extern IPAddress gateway;
extern IPAddress subnet;
extern IPAddress primaryDNS;
extern IPAddress secondaryDNS;
extern uint8_t cap_nb_images;
extern uint8_t cap_interval_dsec;
extern uint8_t cap_size;
extern uint8_t cap_jpg_comp;
extern char latitude[];
extern char longitude[];
extern uint8_t pas_de_veille;
extern uint8_t im_x_debut;
extern uint8_t im_x_fin;
extern uint8_t im_y_debut;
extern uint8_t im_y_fin;

extern const size_t PARAMS_COUNT;
extern Param PARAMS[];

//  -------  CONFIGURATION DES PINS
//  -----------------------------------------------

/* PINOUT DOIT :
0: pour programmation
1: TX pour prog & debug
2: OUT2 : sortie PAC
3: RX pour prog & debug
4: OUT1 : sortie Pompe
5(pin4:markIO35): OUT3 : sortie Arret
12: Btn_reveil
14: SDA Eclairs
15: DSB1820 capteur temp piscine
17: SCL Eclairs
21-22 : SDA, SCL HDC1080
32: IN:capteur DHT22 N°2
33: IN:capteur DHT22 N°1
35:(pin17) (in only) IN : interruption éclairs
36: (in only) IN : capteur pression
39: (in only) IN
*/
/*#define BTN1 14  // Defaut secteur (pullup)
#define BTN2 12  // intrusion    (pullup)
#define BTN3 14  // autoprotection    (pullup)
#define BTN4 15  // marche/Arret    (no pull)  0V:arret 12V:marche
#define BNT5 16  // Reset pour Accesspoint*/

#ifdef MODE_WT32  // WT32_Eth01
// const int PIN_Tint = 11;   // GPIO IN1 Temp interieure DS18B20
const int PIN_Tint22 = 5;  // GPIO IN1 Temp interieure DHT22
const int PIN_PAC = 4;     // GPIO OUT PAC PWM
const int PIN_Text = 36;   //  Text:Entrée analogique 32 à 36 et 39
#else                      // ESP32_DevKit
// const int PIN_Tint = 13;  Défini dans le fichier appli.ino
const int PIN_Tint22 = 5;  // GPIO IN1 Temp interieure DHT22
const int PIN_PAC = 4;     //  OUT PAC - PWM  40kOhm+100nF(Fc=40Hz) et PWM=40khz
#define PIN_Vbatt 0        // Pin Surveillance Batterie (LiPo/2)

// Pin Reveil
#ifdef ESP32_v1
  #define PIN_REVEIL 12  // Pin de réveil (Bouton externe)
#endif
#ifdef ESP32_Fire2    // Firebeetle
  #define PIN_REVEIL 4  // Pin de réveil (Bouton externe) PIN RTC : 0 à 7
#endif
#ifdef ESP32_uPesy
  #define PIN_REVEIL 34  // Pin de réveil (Bouton externe)
#endif
#ifdef ESP32_S3
  #define PIN_REVEIL 12  // Pin de réveil (Bouton externe)
#endif

// Structure d'un message uart
#define MSG_SIZE 40
typedef struct {
  char message[MSG_SIZE];
  uint16_t length;
} UartMessage;

typedef struct {
  uint16_t longueur;  // longueur
  char msg[MSG_SIZE];
} UartMessage_t;

float readBatteryVoltage();
void lectureHeure();
void requete_status(char* json_response, uint8_t socket, uint8_t type);
void recep_message1(UartMessage_t* messa);  // recept_uart1
void maj_etat_chaudiere_delai(uint8_t delai);
void modif_timer_cycle(void);
void traitement_rx(UartMessage_t* mess);
uint8_t requete_Get_appli(const char* var, float* valeur);
#ifdef __cplusplus
uint8_t requete_Set_appli(String param, float valf);
#else
uint8_t requete_Set_appli(const char* param, float valf);
#endif
uint8_t requete_GetReg(int reg, float* valeur);
void capture_video_sd();
void capture_photo_sd();

// Capture AVI in background (no HTTP response). Implemented in app_httpd.cpp
void capture_avi_b();

// encode_lpc is a C++ function; only declare for C++ compilation
#ifdef __cplusplus
int encode_lpc(const lpc_settings_t &settings);
#endif
void printMemoryStatus();

void passage_deep_sleep(uint64_t temps);

extern float Vbatt_Th;   // Tension batterie thermomètre
extern bool Vbatt_Th_I;  // indicateur de réception batt sonde
extern unsigned long last_remote_Text_time, last_remote_Tint_time, last_remote_heure_time;
extern struct tm timeinfo;

extern uint8_t num_err_queue[];

// ESP32-C6 : pins restant à 0 au reset et au boot : 2, 3, 4, 6, 7, 14
const int PIN_Chaudiere = 2;
const int PIN_Text = 36;  //  Text:Entrée analogique 32 à 36 et 39
#ifdef ESP32_v1
const int PIN_RXModbus = 16;  // s3:18  devkitv1:16 RO
const int PIN_TXModbus = 17;  // s3:17  devkitv1:17 DI
#endif
#ifdef ESP32_Fire2
const int PIN_RXModbus = 18;  // s3:18  devkitv1:16 RO
const int PIN_TXModbus = 17;  // s3:17  devkitv1:17 DI
#endif
#ifdef ESP32_uPesy
const int PIN_RXModbus = 16;  // s3:18  devkitv1:16 RO
const int PIN_TXModbus = 17;  // s3:17  devkitv1:17 DI
#endif
const int PIN_on = 19;  // allumage et extinction d'un système avec 2 boutons
const int PIN_off = 19;
// const int PIN_RE = 32;
// const int PIN_DE = 33;
const int PIN_RXSTM = 18;  // RX STM32
const int PIN_TXSTM = 17;  // TX STM32
#endif
// #define sorties analogique : 25 ou 26 (avec Dacwrite)

// Modbus
#ifdef ESP32_v1
#define MAX485_RE_NEG 32  // S3:35  devkitv1:32
#define MAX485_DE 33      // s3:36  devkitv1:33
#endif
#ifdef ESP32_Fire2
#define MAX485_RE_NEG 35  // S3:35
#define MAX485_DE 36      // s3:36
#endif
#ifdef ESP32_uPesy
#define MAX485_RE_NEG 32  // S3:35  devkitv1:32
#define MAX485_DE 33      // s3:36  devkitv1:33
#endif

/* ESP32S3 : Serial0:Pin 42 et 43
 */

typedef enum {
  EVENT_NONE = 0,
  EVENT_INIT,
  EVENT_UART,
  EVENT_ERREUR,
  EVENT_GPIO_ON,
  EVENT_GPIO_OFF,
  EVENT_ECOUTE_WebSock,
  EVENT_WATCHDOG,
  EVENT_24H,
  EVENT_3min,
  EVENT_CYCLE,
  EVENT_PRISE_VIDEO,
  EVENT_PRISE_PHOTO,
  EVENT_ENCODE_LPC,
  EVENT_DECODE_LPC,
  EVENT_UART1
} systeme_eve_type_t;

// Structure d'un événement tache sequenceur
typedef struct {
  systeme_eve_type_t type;  // Type d'événement
  uint32_t data;            // Donnée associée (ex: valeur capteur, byte UART)
} systeme_eve_t;


/* Codes erreur*/
#define Code_erreur_Tint 1
#define Code_erreur_Text 2
#define Code_erreur_Heure 3
#define Code_erreur_depass_tab_status 4
#define Code_erreur_queue_full 5
#define Code_erreur_Json 5
#define Code_erreur_queue 6
#define Code_erreur_google 7
#define Code_erreur_http_local 8
#define Code_erreur_wifi 9
#define Code_erreur_esp_now 10

#define DEBOUNCE_INTERVAL 300  // Temps anti-rebond en ms

#ifdef __cplusplus
constexpr int NB_Graphique = 6;  // Temp Ext, Temp int, Chaud, MoyText, MoyTint, Cout,
constexpr int NB_Val_Graph = 99;
#else
#define NB_Graphique 6
#define NB_Val_Graph 99
#endif

extern uint8_t protocole;
extern uint16_t nb_reset;
extern QueueHandle_t eventQueue;  // File d'attente des événements sequenceur
extern uint16_t erreur_queue;
extern TimerHandle_t debounceTimer;
extern RTC_DATA_ATTR uint8_t periode_cycle;
extern RTC_DATA_ATTR uint8_t mode_rapide;
extern RTC_DATA_ATTR uint16_t prolong_veille;

#define MAX_DUMP 6900              // 600 + 1050 car par graphique
extern char buffer_dmp[MAX_DUMP];  // max 250 logs, 16 octets chacun

extern uint16_t date_ac;
extern uint8_t cpt_securite;
extern uint8_t WIFI_CHANNEL;
extern RTC_DATA_ATTR uint8_t
    last_wifi_channel;     // Mémorisation du canal Wifi en DeepSleep
extern uint8_t rtc_valid;  // 0:cold reset  1:reset apres deep sleep
extern RTC_DATA_ATTR uint16_t   cpt_cycle_batt;                   // Compteur cycles pour mesure batterie
extern volatile uint8_t ackReceived;  // global pour indiquer que le peer a acké
extern volatile int ackChannel;       // canal où ça a marché
extern uint8_t init_time;
extern float heure;
extern RTC_DATA_ATTR uint8_t skip_graph;
extern RTC_DATA_ATTR uint16_t err_Tint, err_Text, err_Heure;  // compteurs d'erreurs
extern float Tint, Text;


extern RTC_DATA_ATTR float tempI_moy24h, tempE_moy24h, cout_moy24h;
extern RTC_DATA_ATTR uint8_t cpt24_Tint, cpt24_Text, cpt24_Cout;

extern char mdp_routeur[];
extern char nom_routeur[];
extern uint8_t DelaiWebsocket;
extern uint8_t log_detail;
extern uint8_t websocket_on;
extern char ip_websocket[];
extern uint8_t id_websocket;
extern RTC_DATA_ATTR uint8_t action_stockage;
extern RTC_DATA_ATTR uint8_t action_envoi;

// IP addresses (RAM objects)
extern IPAddress local_ip;
extern IPAddress gateway;
extern IPAddress subnet;
extern IPAddress primaryDNS;
extern IPAddress secondaryDNS;

extern RTC_DATA_ATTR int16_t graphique[NB_Val_Graph][NB_Graphique];
extern uint16_t Seuil_batt_sonde;  // millivolt
extern uint16_t Seuil_batt_arret_ESP;  // millivolt
extern uint8_t type_reveil;  //0:pas de reveil 1: réveil par timer, 2: réveil par bouton_reveil 3:reveil par PIR
extern uint8_t compteur_graph;

extern RTC_DATA_ATTR uint8_t etat_now;
extern RTC_DATA_ATTR uint8_t Nb_jours_Batt_log;

extern bool force_stay_awake;
extern unsigned long wake_up_time;  // Temps de réveil/dernière activité
extern uint8_t sdcard_ok, camera_ok;

void writeLog(uint8_t code, uint8_t c1, uint8_t c2, uint8_t c3,
              const char* message);
void debounceCallback(TimerHandle_t xTimer);
uint16_t crc16_arc(const uint8_t* data, size_t length);
void log_erreur(uint8_t code, uint8_t valeur,
                uint8_t val2);  // Code:1:Tint, 2:Text, 3:TPac;
void init_10_secondes();
void setup_0();
void setup_nvs();
void setup_1();
void setup_2();
uint8_t requete_action_appli(const char* reg, const char* data);
void appli_event_on(systeme_eve_t evt);
void appli_event_off(systeme_eve_t evt);
#ifdef __cplusplus
uint8_t requete_Get_appli(String var, float* valeur);
uint8_t requete_Set_appli(String param, float valf);
uint8_t requete_GetReg_appli(int reg, float* valeur);
uint8_t requete_SetReg_appli(int param, float valeurf);
uint8_t requete_Get_String_appli(uint8_t type, String var, char* valeur);
uint8_t requete_Set_String_appli(int param, const char* texte);
#else
uint8_t requete_Get_appli(const char* var, float* valeur);
uint8_t requete_Set_appli(const char* param, float valf);
uint8_t requete_GetReg_appli(int reg, float* valeur);
uint8_t requete_SetReg_appli(int param, float valeurf);
uint8_t requete_Get_String_appli(uint8_t type, const char* var, char* valeur);
uint8_t requete_Set_String_appli(int param, const char* texte);
#endif
uint8_t lecture_Tint(float* mesure);
uint8_t lecture_Text(float* mesure);
void event_mesure_temp();
void maj_etat_chaudiere();
void event_mesure_compresseur();

// Fonctions WiFi
uint8_t connectWiFiWithDiagnostic();
void diagnoseWiFiError();
void protectUARTDuringWiFi();

// Configuration DHT22
#define DHT22_TIMEOUT_MS 5000       // Timeout de lecture DHT22 en millisecondes
#define DHT22_MIN_INTERVAL_MS 2000  // Intervalle minimum entre lectures DHT22

#endif