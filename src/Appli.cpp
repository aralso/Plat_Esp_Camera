
#include <Arduino.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "variables.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>  // pour nvs eeprom
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <DHT.h>

#ifdef SDCARD
  #include "SDMMC.h"
#endif
#include "camera.h"


// Seeed Studio XIAO ESP32S3 Sense
// Waveshare ESP32-S3-CAM
// OV3660 : 2048×1536 (3 MP) 1/5" DVP 68° 8/10-bit Raw RGB data
// JPEG compression, YUV/YCbCr422, RGB565
// OV2660

extern WiFiClient client;
extern Preferences preferences_nvs;  // Déclaration externe
extern uint8_t My_statut; // 0:inconnu 1:veille 2:balise 3:tj actif
extern uint16_t nb_err_reseau;
extern  uint16_t TextV, TintV, HumV, HAV;  // pour stockage dans la partition log_flashG
extern volatile uint8_t ackExpectedSequence;
extern TimerHandle_t gatewayQueueTimer;

RTC_NOINIT_ATTR uint8_t etat_ESP_stop;  // 0:normal 1:arrêté pour batterie faible (deepsleep longue duree)
RTC_NOINIT_ATTR uint8_t cpt24h_batt;
RTC_NOINIT_ATTR float PIR_24h=0;
RTC_NOINIT_ATTR uint16_t cpt24_PIR=0, PIRV=0;
RTC_NOINIT_ATTR uint32_t envoi_6s_prec=0, Mesure_6s_prec;
RTC_NOINIT_ATTR uint32_t Tick_6s;
RTC_NOINIT_ATTR float TintPrec=20;

RTC_NOINIT_ATTR uint8_t  WIFI_CHANNEL;
RTC_NOINIT_ATTR uint16_t Seuil_batt_sonde;  // millivolt
RTC_NOINIT_ATTR uint8_t Nb_jours_Batt_log;
RTC_NOINIT_ATTR uint8_t freq_envoi, cpt_mesure, cpt_nb_val;
RTC_NOINIT_ATTR uint16_t valTemp[NB_VAL_TAB], valHum[NB_VAL_TAB];
RTC_NOINIT_ATTR uint32_t valTick[NB_VAL_TAB];

RTC_NOINIT_ATTR uint8_t compteur_graph;
RTC_NOINIT_ATTR uint16_t compteur_24h;

constexpr uint8_t  PIR_NB_TS  = 32;     // taille max de l'historique
static RTC_NOINIT_ATTR uint32_t pir_ts[PIR_NB_TS];  // horodatages (s, horloge RTC)
static RTC_NOINIT_ATTR uint8_t  pir_head;           // prochaine case d'écriture


RTC_NOINIT_ATTR uint8_t Capt_tps_max;
RTC_NOINIT_ATTR uint8_t Capt_seuil_temp;
RTC_NOINIT_ATTR uint8_t Capt_nb_val_max;
RTC_NOINIT_ATTR uint16_t Capt_tps_total_max;
RTC_NOINIT_ATTR uint8_t delai_detection;

// variables Detection PIR
RTC_NOINIT_ATTR uint16_t cpt_detection_PIR_fenetre=0;


RTC_NOINIT_ATTR uint8_t pause_detection;
RTC_NOINIT_ATTR unsigned long last_detection_time=0;


uint8_t code_nb_images=0;
uint8_t photo_video_en_cours=0;
uint8_t battery_level=0;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);

//#if defined(ARDUINO_ARCH_ESP32) && defined(WIFI_TX_INFO_T)
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
/*#else
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
#endif*/

uint8_t parseMacString(const char* str, uint8_t mac[6]);
float absoluteHumidity(float temperature, float relativeHumidity);
void traitement_message_recu(Message_Struct &msg);
uint8_t enreg_valeur();





#ifdef Temp_int_DS18B20
  OneWireNg_CurrentPlatform ow(PIN_DS18B20, false);
  OneWireNg_DS18B20 sensor(&ow);

  // Capteur temperature Dallas DS18B20  Temperature intérieure
  typedef uint8_t DeviceAddress[8];
  const int PIN_Tint = 13;      // Tint:Entrée onewire GPIO DS18B20
  //OneWire oneWire(PIN_Tint);
  //DallasTemperature ds(&oneWire);
  int nb_capteurs_temp = 1;  //DS18B20
  DeviceAddress Thermometer[5];
  DeviceAddress adds;
#endif

#ifdef Temp_int_HDC1080
  ClosedCube_HDC1080 hdc1080;
#endif

// Temperature intérieure
float Tint, Text, Humid;

uint16_t err_Tint, err_Text, err_Heure;  // compteurs d'erreurs


// Entrée analogique pour mesurer la température exterieure
uint16_t Text1, Text2, Text1Val, Text2Val;  // valeurs pour calibration (T*10)

#define resolutionADC 4095
#define TBETA 3950  // Coefficient Beta de la thermistance
uint16_t TBeta;
#define R0E 10000  // Résistance à 25° pour une NTC10K
uint16_t Therm0;

uint16_t S_analo_max;  // voltage pour 20°C
float Teau = 18;
float TRef = 18;

#define TPACMIN 10
#define TPACMAX 30
uint8_t TPacMin, TPacMax;

// Régulation
#define MODE 1  // 1:STM32 PID  2:ESP PID
uint8_t Mode;

int readLastLogsG(int nombre);

#ifdef Temp_int_DS18B20
  void printAddress(DeviceAddress deviceAddress) {
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print("0x");
      if (deviceAddress[i] < 0x10) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
      if (i < 7) Serial.print(", ");
    }
    Serial.println("");
  }
#endif

void init_10_secondes()
{
}

//setup au debut
void setup_0()
{

  /*if (NB_Graphique==6)
  {
    graphique[0][0] = 180;  //Tint - vert
    graphique[1][0] = 185;
    graphique[2][0] = 190;

    graphique[0][1] = 110;  // Text - bleu
    graphique[1][1] = 80;
    graphique[2][1] = 103;
    graphique[3][1] = 95;

    graphique[0][2] = 150;  // Chaud
    graphique[1][2] = 150;
    graphique[2][2] = 200;
    graphique[3][2] = 200;

    graphique[0][3] = 185;  // Tint moy
    graphique[1][3] = 183;
    graphique[2][3] = 183;
    graphique[3][3] = 195;

    graphique[0][4] = 35;   // Text moy
    graphique[1][4] = 38;
    graphique[2][4] = 42;
    graphique[3][4] = 32;

    graphique[0][5] = 50;  // cout
    graphique[1][5] = 55;
    graphique[2][5] = 48;  
    graphique[3][5] = 52;
  }*/
}


// setup apres la lecture nvs, avant démarrage reseau
void setup_1()
{
  // initialisation capteur de température intérieur
    Tint = 15;
    #ifdef Temp_int_DHT22
      dht[0].begin();
    #endif
 
    #ifdef Temp_int_HDC1080

      #ifdef ESP32_v1
        Wire.begin(21, 22); // Forçage des pins SDA=21, SCL=22 pour ESP32 DevKit V1
      #endif
      #ifdef ESP32_Fire2
        Wire.begin(19, 20); // Forçage des pins SDA=20, SCL=21 pour ESP32 Firebeetle 2
      #endif
      #ifdef ESP32_uPesy
        Wire.begin(21, 22); // Forçage des pins SDA=21, SCL=22 pour ESP32 uPesy
      #endif

      hdc1080.begin(0x40);
      /*if (i2cDevicePresent(0x40)) {
        Serial.println("HDC1080 détecté");
        hdc1080.begin(0x40);
      } else {
        Serial.println("HDC1080 ABSENT");
      }*/
    #endif

 
    #ifdef Temp_int_DS18B20
      ds.begin();  // Startup librairie DS18B20
      nb_capteurs_temp = ds.getDeviceCount();
      Serial.print("Nb Capteurs DS18B20:");
      Serial.println(nb_capteurs_temp);
      if (nb_capteurs_temp > 1) nb_capteurs_temp = 1;
      int j;
      for (j = 0; j < nb_capteurs_temp; j++) {
        Serial.print(" Capteur :");
        ds.getAddress(Thermometer[j], j);
        printAddress(Thermometer[j]);
      }
    #endif

    // lecture initiale temperature interieure
    /*uint8_t Tint_err = lecture_Tint(&Tint);
    if ((Tint < 1) || (Tint > 45)) {
      Tint = 20.0;
      Tint_err = 7;
    }
    if (Tint_err) log_erreur(Code_erreur_Tint, Tint_err, 1);
    else
      Serial.printf("Temp int:%.2f\n\r", Tint);  */
}

// apres demarrage reseau
void setup_2()
{

  // lecture des données sauvegardées dans la partition log_flashG
  readLastLogsG(99);

  #ifndef ESP_VEILLE  // Toujours actif

    // Configuration WiFi en mode Station pour ESP-NOW

    if ((mode_reseau==13) )
      WiFi.mode(WIFI_STA);
    
    // 🔍 DIAGNOSTIC: Forcer le canal WiFi
    uint8_t current_channel;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&current_channel, &second);
    Serial.printf("Canal WiFi AVANT config ESP-NOW: %d\n\r", current_channel);
    
    // Forcer le canal si nécessaire (doit correspondre au routeur)
    // esp_wifi_set_promiscuous(true);
    // esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    // esp_wifi_set_promiscuous(false);
    
    if (esp_now_init() != ESP_OK) {
      Serial.println("Erreur initialisation ESP-NOW");
      return;
    }
    //if ((mode_reseau==13) )
    //else
    //  Serial.println(WiFi.softAPmacAddress());
    Serial.println(WiFi.macAddress());

    // Stockage de l'adresse MAC dans le tableau mac_gw[6]
    Serial.printf("   MAC dest : %02X:%02X:%02X:%02X:%02X:%02X\n\r",
            mac_gw[0], mac_gw[1], mac_gw[2],
            mac_gw[3], mac_gw[4], mac_gw[5]);


    Serial.printf("   Canal WiFi: %d\n\r", current_channel);
    Serial.println("   En attente de messages...");
    Serial.println("======================================\n\r");
    delay(2000); // 2 secondes de pause pour lire

  #endif
}



void appli_event_on(systeme_eve_t evt)
{
}

void appli_event_off(systeme_eve_t evt)
{
  // Detecteur PIR activé
  if (evt.data == 1)
  {
    detection_pir();
  }

}

void setupRoutes_appli()
{
}

void init_rtc_variables_appli()  // initialisation si perte d'alim
{
  num_sequentiel=0;
  pir_head = 0;
  cpt_detection_PIR_fenetre = 0;
  PIR_24h=0;
}

void init_ram_variables_appli()  // initialisation à chaque démarrage/reset/réveil
{
}


uint8_t setup_appli()
{
  uint8_t continue_setup=1;  // continue le setup après return sauf si 0

  if (!rtc_valid)  // initialisation si perte d'alim
  {
    last_wifi_channel = WIFI_CHANNEL;  // prochain canal WiFi à utiliser après perte d'alim
    Serial.printf("Prochain canal WiFi a utiliser apres perte d'alim: %d\n\r", last_wifi_channel);
  }
  
  // -------------  Capteur/Detecteur : stockage ou envoie infos à la gateway -------------------

  if (type_reveil ==2)   // Detection PIR
  {
    detection_pir();
    uint64_t sleep_time = (uint64_t)periode_cycle * 60 * 1000000;
    if (mode_rapide==12)
    sleep_time = (uint64_t)periode_cycle * 1000000;
    passage_deep_sleep( sleep_time, 10); // au power-on apres detection PIR
  }
  else 
  {    
    if (type_reveil == 4)    // BTN => type_reveil=4
    {  
      // La valeur de prolong_veille est chargée depuis la NVS plus bas dans setup().
      // Recalculer l'échéance ici évite d'utiliser une valeur RTC obsolète.
      wake_up_time = millis() + prolong_veille*1000;
      delay(100);  // délai pour différencier appui court de appui long
      uint8_t etat_BTN0 = digitalRead(BTN_PIN[0]);  // repos-pull_up => 1, appui => 0
      if (log_detail>=1)Serial.printf("Etat BTN0: %i  type_reveil:%i\n\r", etat_BTN0, type_reveil);
      if (!etat_BTN0)  // BTN encore appuyé
      {
        continue_setup=1;
        My_statut = 3;
      }
      else 
      {
        continue_setup=0;
      }
    }
    else continue_setup=0;


    if (type_reveil == 1)    // reveil par timer
    {
      event_cycle(); // 
      continue_setup=0;  // ne continue pas le setup après réveil par timer
    }
    else // BTN ou power on
    {
      if (log_detail>=2) Serial.printf("milli H1: %lu\n\r", millis());
      envoi_temp_hygro();  // 30ms
       if (log_detail>=2) Serial.printf("milli H3: %lu\n\r", millis());
    }
    
    if (!continue_setup)  
    {
      if (gatewayQueueTimer != NULL && xTimerIsTimerActive(gatewayQueueTimer) != pdFALSE)
      {
          if (log_detail>=3) Serial.println("Le timer Envoi_now tourne encore => pas de veille");
      }
      else  // 
      {
        uint64_t sleep_time = (uint64_t)periode_cycle * 60 * 1000000;
        if (mode_rapide==12)
        sleep_time = (uint64_t)periode_cycle * 1000000;
        passage_deep_sleep( sleep_time, 11); // apres power-on suite mesure temperature/hygrometrie
      }
    }
  }
  return continue_setup;
}

void setup_3()
{
}

void traitement_recalage_espnow(uint8_t node)
{
}

// log batterie et Temp moyenne toutes les 24h.
void enreg_24h( uint8_t veille)
{
    // Log toutes les jours/semaines : nb d'erreurs wifi et batteries
  float vbatt = readBatteryVoltage();
  if (log_detail>=1) Serial.printf("24h - Nb jours batt log: %i vbatt:%.2f\n\r", Nb_jours_Batt_log, vbatt);

  if (Nb_jours_Batt_log)
  {
    if (log_detail>=3) Serial.printf("Tension Batterie : %.2f V\n\r", vbatt);

    if ((Seuil_batt_arret_ESP) && (vbatt < Seuil_batt_arret_ESP) && (vbatt > 2400))
    {
      writeLog('S', 7, 0, 0, "Batt Stp");
      etat_ESP_stop = 1;
    }
    cpt24h_batt++;
    if (cpt24h_batt >= Nb_jours_Batt_log)  // log chaque X jour 
    {
      cpt24h_batt=0;

      //Serial.printf("Tension Batterie 24h : %.2f %.2f V\n\r", vbatt, Vbatt_Th);
      if (veille)
      {
        writeLog('K', (uint8_t)((vbatt-2.0)*100), 0, 0, "24H_Res");
      }
      else
      {
        if (nb_err_reseau>255) nb_err_reseau=255;
        writeLog('K', (uint8_t)((vbatt-2.0)*100), 0, (uint8_t)nb_err_reseau, "24H_Res");
        nb_err_reseau=0;
      }
    }
  }

  // erreurs Tint, Text, heure
  if (err_Tint>254) err_Tint=255;
  if (err_Tint)
            log_erreur(Code_erreur_Tint, err_Tint,0);
  if (err_Text>254) err_Text=255;
  if (err_Text)
            log_erreur(Code_erreur_Text, err_Text,0);
  if (err_Heure>254) err_Heure=255;
  if (err_Heure)
            log_erreur(Code_erreur_Heure, err_Heure,0);
  err_Tint=0;
  err_Text=0;
  err_Heure=0;

  // graphique des temperatures quotidiennes
  uint16_t tempI=1, tempE=1, Hum=1, HA=1, PIR=1;
  if (cpt24_Tint)  tempI = (uint16_t)(tempI_moy24h/cpt24_Tint*10);
  Serial.printf("tempI: %i cpt24_Tint: %i Temp24h I:%.2f \n\r", tempI, cpt24_Tint, tempI_moy24h);
  if (cpt24_Text)  tempE = (uint16_t)(tempE_moy24h/cpt24_Text*10);
  if (cpt24_Hum) Hum = (uint16_t)(Hum_24h/cpt24_Hum*10);
  if (cpt24_HA) HA = (uint16_t)(HA_moy24h/cpt24_HA*10);
  PIR = (uint16_t)(PIR_24h);
  if (!tempI) tempI=1;  // permet d'afficher quand meme le point sur le graphique
  if (!tempE) tempE=1;  // permet d'afficher quand meme le point sur le graphique
  if (!Hum) Hum=1;  // permet d'afficher quand meme le point sur le graphique
  if (!HA) HA=1;  // permet d'afficher quand meme le point sur le graphique
  if (!PIR) PIR=1;  // permet d'afficher quand meme le point sur le graphique
  TextV = tempE;
  TintV = tempI;
  HumV = Hum;
  HAV = HA;
  PIRV = PIR;

  //Serial.printf("Temp24h I:%.2f %i E:%.2f %i C:%.2f %i\n\r", tempI_moy24h, cpt24_Tint, tempE_moy24h, cpt24_Text, cout_moy24h, cpt24_Cout);

  tempI_moy24h=0;
  tempE_moy24h=0;
  Hum_24h=0;
  HA_moy24h=0;
  PIR_24h=0;
  cpt24_Tint=0;
  cpt24_Text=0;
  cpt24_Hum=0;
  cpt24_HA=0;
  cpt24_PIR=0;

  uint8_t i;
  for (i = NB_Val_Graph - 1; i; i--) {
    graphique[i][3] = graphique[i - 1][3];
    graphique[i][4] = graphique[i - 1][4];
    graphique[i][5] = graphique[i - 1][5];
  }
  graphique[0][3] = tempI;
  graphique[0][4] = selec_graph(GRAPH2, HA, Hum, PIR);
  graphique[0][5] = selec_graph(GRAPH3, HA, Hum, PIR);  

  if (log_detail>=1) Serial.printf("Graphique 24h : Tint:%i Text:%i Hum:%i HA:%i\n\r", tempI, tempE, Hum, HA);
  writeLogG('G', tempI, HA, Hum); // Enregistrment en Flash des 3 valeurs du graphique

  // Tick, Temp, Hum, Vbatt
  if (action_envoi && esp_now_actif)
  {
    Message_Struct message;

    message.destinataire = SERVER_ADD | 0x80;  // 0x80 = message hexa
    message.emetteur = My_Address;
    message.statut = 0b01;  // mode veille
    if (My_statut == 3)  message.statut = 0b11; // mode actif
    message.statut &= 0b10111;  // Ack demandé bit3=0;
    num_sequentiel++;
    message.num_seq = num_sequentiel;
    message.code = 'C';
    message.code2 = 'J';

    uint8_t pos = 0;
    message.payload[pos++] = (uint8_t) Tick_6s;
    message.payload[pos++] = (uint8_t) (Tick_6s >> 8);
    message.payload[pos++] = Tick_6s >> 16;
    payloadWrite(message.payload, pos, tempI);
    payloadWrite(message.payload, pos, Hum);
    payloadWrite(message.payload, pos, (uint16_t)(vbatt*100));
    message.longueur = 4 + pos;

    uint8_t result = envoi_data(message, 0);  // envoi chaque 24h des valeurs à la gateway

  }      
}

void event_cycle()  // toutes les 10 minutes  (Power on, Timer on, inconnu)
{

  // timeout sur le cycle en cours
  if (photo_video_en_cours) photo_video_en_cours--;

  if (mode_rapide)  Tick_6s += ((uint16_t)periode_cycle)/6; // ajouter le temps correspondant 
  else  Tick_6s += (uint16_t)periode_cycle*10;


  // Récupération de la température extérieure par internet
  //fetch_internet_temp();
  Text = 0;

  // lecture temp-humi
  uint8_t err_Tint = lecture_Tint(&Tint, &Humid);
  if (err_Tint)
    log_erreur(Code_erreur_Tint, err_Tint, 1);
  else
    if (log_detail>=2) Serial.printf("Temp int:%.2f Humid:%.2f\n\r", Tint, Humid); 

  float HA = absoluteHumidity(Tint, Humid);

  float tempI_moy15m, tempE_moy15m, HA_moy15m, Hum_moy15m, PIR_moy15m;
  uint8_t cpt15_Tint, cpt15_Text, cpt15_HA, cpt15_Hum, cpt15_PIR;

  tempI_moy15m += Tint;
  cpt15_Tint++;
  tempE_moy15m += Text;
  cpt15_Text++;
  HA_moy15m += HA;
  cpt15_HA++;
  Hum_moy15m += Humid;
  cpt15_Hum++;
  PIR_moy15m += cpt_detection_PIR_fenetre/4;
  cpt15_PIR++;

  // chaque 15 minutes
  compteur_graph++;
  if (compteur_graph >= skip_graph)  // 1 valeur sur x
  {
    int16_t tempI_arrondi = 200;
    int16_t tempE_arrondi = 150;
    int16_t HA_arrondi = 10;
    int16_t Hum_arrondi = 500;
    int16_t PIR_arrondi = 0;

     if (cpt15_Tint > 0) {  tempI_arrondi = round(tempI_moy15m / cpt15_Tint * 100);  }
     if (cpt15_Text > 0) {  tempE_arrondi = round(tempE_moy15m / cpt15_Text * 100); }
     if (cpt15_HA > 0) {    HA_arrondi = round(HA_moy15m / cpt15_HA * 100); }
     if (cpt15_Hum > 0) {   Hum_arrondi = round(Hum_moy15m / cpt15_Hum * 100); }
     if (cpt15_PIR > 0) {   PIR_arrondi = round(PIR_moy15m / cpt15_PIR * 10); }

    compteur_graph = 0;
    for (uint8_t i = NB_Val_Graph - 1; i; i--) {
      graphique[i][0] = graphique[i - 1][0];
      graphique[i][1] = graphique[i - 1][1];
      graphique[i][2] = graphique[i - 1][2];
    }
    graphique[0][0] = tempI_arrondi; // 20°C => 2000
    graphique[0][1] = selec_graph(GRAPH2, HA_arrondi, Hum_arrondi, PIR_arrondi);
    graphique[0][2] = selec_graph(GRAPH3, HA_arrondi, Hum_arrondi, PIR_arrondi);


    if (action_envoi) 
    {
      if (freq_envoi)  // envoi periodique activé
      {
        cpt_mesure++;
        if (cpt_mesure >= freq_envoi)  // mesure toutes les x valeurs (x*skip_graph*15 minutes)
        {
          if (log_detail>=3) Serial.printf("Mesure periodique toutes les %i valeurs (cpt_mesure=%i)\n\r", freq_envoi, cpt_mesure);
          cpt_mesure=0;
          enreg_valeur();  // enreg puis envoi  par ESP-NOW
          TintPrec = Tint;
        }
      }
      else // envoi lorsque les valeurs évoluent
      {
        uint8_t enreg=0;
        float diffTint = (Tint - TintPrec)*100;  // en 0,01°C
        if (diffTint > Capt_seuil_temp) enreg=1;
        Serial.printf("Temp Int: %.2f , Temp Int Prec: %.2f , Seuil: %i\n\r", Tint, TintPrec, Capt_seuil_temp);
        if (-diffTint > Capt_seuil_temp) enreg=1;
        if ((Tick_6s - Mesure_6s_prec) > Capt_tps_max*10) enreg=1;
        Serial.printf("Tick_6s : %i, Mesure_6s_prec: %i, Capt_tps_max: %i\n\r", Tick_6s, Mesure_6s_prec, Capt_tps_max);
        if (enreg) {
          Serial.printf("Envoi  (cpt_nb_val=%i)\n\r", cpt_nb_val);
          enreg_valeur();  // enreg puis envoi  par ESP-NOW
          TintPrec = Tint;
          Mesure_6s_prec = Tick_6s;
        }
      }
    }
    tempI_moy15m = 0;
    tempE_moy15m = 0;
    HA_moy15m = 0;
    Hum_moy15m = 0;
    PIR_moy15m = 0;
    cpt15_Tint = 0;
    cpt15_Text = 0;
    cpt15_HA = 0; 
    cpt15_Hum = 0;
    cpt15_PIR = 0;
  }
  tempI_moy24h += Tint;
  cpt24_Tint++;
  tempE_moy24h += Text;
  cpt24_Text++;
  HA_moy24h += HA;
  cpt24_HA++;
  Hum_24h += Humid;
  cpt24_Hum++;
  //PIR_24h += cpt_detection_PIR_fenetre/4; // chaque 15 min
  //cpt24_PIR++;

  //Serial.printf("fin cycle :reveil:%i cpt:%i %i tint:%i 24h:%i\n\r", type_reveil, compteur_graph, skip_graph, graphique[0][0], compteur_24h);
  if (log_detail>=1) Serial.printf("compteur 24h:%i\n\r", compteur_24h);

  if (type_reveil != 10)      // si diff de toujours actif => compteur 24h
  {
    compteur_24h++;
    if (compteur_24h >= (uint16_t)24*60/periode_cycle) // )  // toutes les 24h
    {
      Serial.println("24h");
      activation_writelog();
      enreg_24h(1);  // et envoi si actif
      compteur_24h=0;
    }
  }
}

// Called from the event loop when EVENT_PRISE_VIDEO is dequeued.
// Delegate actual AVI capture to the background capture function implemented
// in the camera HTTP module so the event task performs the long-running work.
uint8_t capture_video_sd()
{
    //Serial.println("capture_video_sd(): event— starting background AVI capture");
    return capture_avi_b();
}

uint8_t eval_action_batt_frequence()
{
   /* determination du nb d'images/video à prendre suite à detection pir
   0 : aucune action
   1 : photo jpg : 1 image
   2 à 9 : video 2img/sec en multiples de 2 : 2-4-8-16-32-64-128-256(2min)
   conditions : état de la batterie et nb de detections récentes durant la fenetre de 10 minutes
   niveau = niveau de base (batterie) - pénalité (détections récentes)
   */
    // Batterie LiPo : 3.3V = 0%, 4.2V = 100% (approximation linéaire)
    float vbatt = readBatteryVoltage();
    int batt = (int)((vbatt - 3.3f) / (4.2f - 3.3f) * 100.0f);
    if (batt < 0) batt = 0;
    if (batt > 100) batt = 100;
    battery_level = batt;

    // cpt_detection_PIR_fenetre inclut déjà la détection courante (fenêtre 1h)
    uint16_t recent = (cpt_detection_PIR_fenetre > 0) ? cpt_detection_PIR_fenetre - 1 : 0;

    // 1. Niveau de base selon la batterie
    int level;
    if      (batt < 10) return 0;   // protection batterie
    else if (batt < 25) level = 1;
    else if (batt < 40) level = 3;
    else if (batt < 55) level = 5;
    else if (batt < 70) level = 6;
    else if (batt < 85) level = 7;
    else                level = 9;

    // 2. Pénalité selon les détections récentes
    if (recent >= 11) return 0;     // rafale : on laisse passer
    int penalty;
    if      (recent <= 1)  penalty = 0;
    else if (recent <= 3)  penalty = 1;
    else if (recent <= 7)  penalty = 2;
    else if (recent <= 9) penalty = 3;
    else                   penalty = 4;
    if (batt < 40) penalty *= 2;    // batterie faible : plus strict

    // 3. Résultat : si la pénalité épuise le niveau, aucune action
    level -= penalty;
    if (level < 1) return 1;
    return (uint8_t)level;
}



void detection_pir()
{
  // determination du nombre de détection PIR durant la dernière heure et 24h
  {
    constexpr uint32_t PIR_FENETRE_S = 3600; // 1 heure

    uint32_t now = (uint32_t)time(nullptr);
    if (pir_head >= PIR_NB_TS || cpt_detection_PIR_fenetre > PIR_NB_TS)  // RTC non initialisée
    {
      pir_head = 0;
      cpt_detection_PIR_fenetre = 0;
    }
    // supprime les plus anciennes détections hors fenêtre (ou postérieures à now : horloge recalée)
    while (cpt_detection_PIR_fenetre > 0)
    {
      uint8_t oldest = (pir_head + PIR_NB_TS - cpt_detection_PIR_fenetre) % PIR_NB_TS;
      uint32_t ts = pir_ts[oldest];
      if (ts > now || (now - ts) > PIR_FENETRE_S) cpt_detection_PIR_fenetre--;
      else break;
    }
    // enregistre la détection courante
    pir_ts[pir_head] = now;
    pir_head = (pir_head + 1) % PIR_NB_TS;
    if (cpt_detection_PIR_fenetre < PIR_NB_TS) cpt_detection_PIR_fenetre++;
  }

  PIR_24h++;

  // determination action photo/video suite à detection_pir
    code_nb_images = eval_action_batt_frequence();
    

    if (!photo_video_en_cours)
    {
      writeLog('P', code_nb_images, battery_level, cpt_detection_PIR_fenetre, "PIR nb I");

      if (code_nb_images==1)
      {
        photo_video_en_cours=3;
        systeme_eve_t evt = { EVENT_PRISE_PHOTO, 0 };  // Exemple : donnée = 123
        if (xQueueSendFromISR(eventQueue, &evt, NULL) != pdTRUE) 
        {
          if (erreur_queue<5) num_err_queue[erreur_queue]=7;
          erreur_queue++;
        }
      }
      if ((code_nb_images>1) && (code_nb_images<10))
      {
        photo_video_en_cours=3;
        systeme_eve_t evt = { EVENT_PRISE_VIDEO, 0 };  // Exemple : donnée = 123
        if (xQueueSendFromISR(eventQueue, &evt, NULL) != pdTRUE) 
        {
          if (erreur_queue<5) num_err_queue[erreur_queue]=8;
          erreur_queue++;
        }
      }
    }
    else
      writeLog('E', code_nb_images, 0, 0, "PIR cour");

}

char* requete_status_appli(char *json_response, char *p, uint8_t type)
{
  p += sprintf(p, "\"PIR_D\":%i,", cpt_detection_PIR_fenetre); // derniere heure
  p += sprintf(p, "\"PIR_V\":%i,", PIRV); // veille
  return p;
}

// type 1
uint8_t requete_Get_appli(const char* var, float *valeur)
  //uint8_t requete_Get_appli (String var, float *valeur) 
{
  uint8_t res=1;

  if (strncmp(var, "Tint",5) == 0) {
    res = 0;
    *valeur = Tint;
  }
  if (strncmp(var, "Text",5) == 0) {
    res = 0;
    *valeur = Text;
  }

  return res;
}


// type 1
uint8_t requete_Set_appli (String param, float valf) 
{
  uint8_t res=1;
  int8_t val = round(valf);
  (void)val;

    if (param == "consigne")     // Forcage consigne, rajouter duree
    {
      if ((valf >= 6.0) && (valf <= 22.0))  // 6°C à 22°C
      {
          res = 0;
      }
    }

  return res;
}

// type 2
uint8_t requete_GetReg_appli(int reg, float *valeur)
{
  uint8_t res = 1;

  // Most numeric parameters are now returned via the generic PARAMS table.
  // Keep here only application-specific dynamic reads that are not present
  // in the PARAMS table.

  if (reg == 41)  // registre 41 : canal WiFi actuel (dynamic)
  {
    res = 0;
    uint8_t current_channel;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&current_channel, &second);
    *valeur = (float)current_channel;
  }

  return res;
}

// type 2
uint8_t requete_SetReg_appli(int param, float valeurf)
{
  int16_t valeur = int16_t(round(valeurf));
  uint8_t res = 1;
  (void)valeur;

  return res;
}

// type 4
uint8_t requete_Get_String_appli(uint8_t type, String var, char *valeur)
{
  uint8_t res=1;
  int paramV = var.toInt();
  // valeur limité a 50 caractères
  
  if (paramV == 60)  // registre 60 : adresse MAC ce module
  {
    res = 0;
    strncpy(valeur, WiFi.macAddress().c_str(), 18);
  }
  if (paramV == 61)  // registre 61 : adresse MAC GW
  {
    res = 0;
    snprintf(valeur, 18,
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_gw[0], mac_gw[1], mac_gw[2],
           mac_gw[3], mac_gw[4], mac_gw[5]);
  }

  return res;
}


// type 4
uint8_t requete_Set_String_appli(int param, const char *texte)
{
  uint8_t res=1;
  IPAddress ip;

    if (param == 61)  // registre 61 : adresse Mac GW
    {
      if (!parseMacString(texte, mac_gw))
      {
          Serial.println("MAC dest invalide");
      }
      else
      {
        preferences_nvs.putString("MacC", texte);
        res = 0;
      }
    }

  return res;
}

// type5 : reception message ACTION par uart ou par page web
uint8_t requete_action_appli(const char *reg, const char *data)
{
  uint8_t res=1;


  if (strcmp(reg, "Photo") == 0) 
    { 
      res=0; 
      systeme_eve_t evt = { EVENT_PRISE_PHOTO, 0 };  // Exemple : donnée = 123
      if (xQueueSendFromISR(eventQueue, &evt, NULL) != pdTRUE) 
      {
        if (erreur_queue<5) num_err_queue[erreur_queue]=3;
        erreur_queue++;
      }
    }

  if (strcmp(reg, "Video") == 0) 
    { 
      res=0; 
      systeme_eve_t evt = { EVENT_PRISE_VIDEO, 0 };  // Exemple : donnée = 123
      if (xQueueSendFromISR(eventQueue, &evt, NULL) != pdTRUE) 
      {
        if (erreur_queue<5) num_err_queue[erreur_queue]=3;
        erreur_queue++;
      }
    }

  if (strcmp(reg, "Tint") == 0) 
    { 
      res=0; 
      float humid;
      uint8_t Tint_erreur = lecture_Tint(&Tint, &humid);
      Serial.println(Tint_erreur);
      Serial.println(Tint);
      Serial.println(humid);
    }

    
  return res;
}


// erreur :0:ok  sinon erreur 2 à 7
uint8_t lecture_Tint(float *mesure, float*humid)
{
  uint8_t Tint_erreur = 7;
  float valeur = 20;
  float valeur2 = 50;

  #ifdef Temp_int_DHT22
    //dht[0].begin();

    if (digitalRead(PIN_Tint22) == HIGH || digitalRead(PIN_Tint22) == LOW)
    {
      valeur =  dht[0].readTemperature();
      if (isnan(valeur))
      {
        valeur = 20.0;
        Tint_erreur = 6;
        Serial.println("---DHT:non numérique");
      }
      else
      {
        Tint_erreur=0;
      }
    }
    else
      Serial.println("---DHT:signal non stable!");
  #endif

  #ifdef Temp_int_HDC1080
    valeur = hdc1080.readTemperature();
    if (isnan(valeur)) {
      valeur = 20.0;
      Tint_erreur = 4;
    } else {
      Tint_erreur=0;
    }
  #endif

  #ifdef Temp_int_DS18B20
    valeur = ds.getTemperature();
    Tint_erreur=0;
  #endif

  if (valeur > 50) Tint_erreur = 2;
  if (valeur < -20) Tint_erreur = 3;
  if (log_detail>=3) Serial.printf("lecture Tint : %.2f Humid:%.2f Tint_erreur:%i\n\r", valeur, valeur2, Tint_erreur);
  if (Tint_erreur) {
    valeur = 20.0;
    valeur2 = 50.0;
  }
  *mesure = valeur;
  *humid = valeur2;
  return Tint_erreur;
}



//mesure temperature exterieure
uint8_t lecture_Text(float *mesure) {
  uint8_t Text_erreur = 0;
  int16_t Val_Text = 1600;
  float valeur;

  #ifdef MODBUS
    Text_erreur = read_modbus(2, &Val_Text);  // registre 1-2-3 pour temp exterieure
    valeur = (float)Val_Text / 10;
  #else
    valeur = 18.0;

    /*#ifndef DEBUG_SANS_Sonde_Ext
        Val_Text = analogRead( PIN_Text );  // 0 à 4096
        //Serial.println(Val_Text);
      #endif*/
    // calibration
    // Text1:100(10°C) Text1Val:500
    // Text2:200(20°C) Text2Val:2000
    //valeur = ((float)(Text1Val-Val_Text)/(Text1Val-Text2Val)*(Text2-Text1) + Text1)/10;

    /*float Vmesure = ((float)Val_Text / resolutionADC) * 3.66;
      float Rntc = 15000 * Vmesure / (3.3 - Vmesure);  // Calcul de la résistance de la thermistance
      float T_kelvin = 1.0 / ((1.0 / 298.15) + (1.0 / TBeta) * log(Rntc / Therm0));    // Calcul de la température en Kelvin
      valeur = T_kelvin - 273.15;    // Conversion en °C */
    //Serial.printf("val_text:%i vmesure:%.3f rntc:%.0f T_kelvin:%.1f valeur:%.1f\n", Val_Text, Vmesure, Rntc, T_kelvin, valeur);
  #endif

  if ((valeur < -30.0) || (valeur > 60.0)) Text_erreur = 1;
  if (!Val_Text) Text_erreur = 2;

  *mesure = valeur;
  return Text_erreur;
}



uint8_t fetch_internet_temp() {

  uint8_t res=1;

  // 1. Vérifier si le réseau est disponible avant de commencer
  if (WiFi.status() != WL_CONNECTED) {
    // Si vous utilisez l'Ethernet, remplacez par le test approprié
    return res; 
  }

  HTTPClient http;

  // 2. Définir un timeout court (2000ms au lieu des 5-10s par défaut)
  http.setTimeout(2000); 

  char url[150];  // assez grand pour contenir toute l'URL
  sprintf(url, "http://api.open-meteo.com/v1/forecast?latitude=%s&longitude=%s&current=temperature_2m", latitude, longitude);


  if (http.begin(url)) {
    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      DynamicJsonDocument doc(512);
      DeserializationError error = deserializeJson(doc, payload);
      
      if (!error) {
        float temp = doc["current"]["temperature_2m"] | NAN;
        if (!isnan(temp) && temp > -50.0 && temp < 60.0)
        {
          res = 0;
          Text = temp;
          //Serial.printf("Météo Garches : %.1f°C\n", Text);
          uint32_t mil = millis();
          if (mil - last_remote_Text_time > 35*60*1000) // le precedent message est vieux de plus de 35 minutes
            err_Text++;
          last_remote_Text_time = mil;
          cpt24_Text++;
          tempE_moy24h += Text;
        }
      } else {
        Serial.printf("Erreur parsing JSON Météo : %s\n", error.c_str());
      }
    } else {
      Serial.printf("Erreur HTTP Météo (%d) : %s\n", httpCode, http.errorToString(httpCode).c_str());
    }
    http.end();
  }
  return res;
}

void event_mesure_temp()  // toutes les 15 minutes
{
  uint8_t i;

  // Récupération de la température extérieure par internet
  fetch_internet_temp();


  compteur_graph++;
  if (compteur_graph >= skip_graph)  // 1 valeur sur x
  {
    compteur_graph = 0;
    for (i = NB_Val_Graph - 1; i; i--) {
      graphique[i][0] = graphique[i - 1][0];
      graphique[i][1] = graphique[i - 1][1];
      graphique[i][2] = graphique[i - 1][2];
    }
    graphique[0][0] = round(Tint * 10);
    graphique[0][1] = round(Text * 10);

    graphique[0][2] = 0;  // 14:arret 16:marche
  }

    // cout moyen
}




float readBatteryVoltage() {
  // Lecture ADC (0-4095) sur PIN_Vbatt
  // Sur ESP32 DevKit V1, l'ADC est calibré par défaut
  int raw = analogRead(PIN_Vbatt);
  
  // Conversion:
  // raw / 4095.0 * 3.3V (tension ref approx) * 2 (pont diviseur) * 1.1 (facteur corection empirique souvent nécessaire sur ESP32)
  // On commence sans facteur 1.1 pour tester
  float voltage = (raw / 4095.0) * 3.3 * 2.5; 
  return voltage;
}


void traitement_message_recu_appli(Message_Struct &msg)
{
  uint8_t mess_valid=0;

  Serial.printf("code1: %i code2:%i\n\r", msg.code, msg.code2);

  if (!mess_valid)
  {
    if (log_detail >=3) Serial.println("message pas valide");
  }

  if (!(msg.statut & 0b1000)) // demande d'ack => envoi ack
  {
    envoi_ack(msg);
  }
}


void traitement_espnow_recv(EspNowRecvMsg_t &recv) {
  Message_Struct &msg = recv.msg;
  int len = recv.len;
  uint8_t *src_addr = recv.src_addr;

  // 🔍 DIAGNOSTIC: Afficher infos de réception
  if (log_detail >=1)  Serial.println("\n📥 ========== RECEPTION ESP-NOW ==========");
  if (log_detail>=3) 
  {
    Serial.printf("   Adresse MAC source: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", src_addr[i]);
      if (i < 5) Serial.print(":");
    }
    Serial.println();
  }
  
  // Afficher le canal WiFi actuel
  uint8_t current_channel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&current_channel, &second);
  if (log_detail>=4) Serial.printf("   Canal WiFi actuel: %d\n\r", current_channel);
  if (log_detail>=3)Serial.printf("   Taille recue: %d octets\n\r", len);
  
  if (len > sizeof(Message_Struct)) {
    Serial.println("⚠️ message trop long");
    return;
  }
  num_sequentiel = msg.num_seq;
  uint8_t renvoi_ack=0;

  if (log_detail>=1)
  {
    Serial.printf("   Donnees recues: ");
    for (uint8_t i = 0; i < len; i++) Serial.printf("%02X ", ((uint8_t*)&msg)[i]);
    Serial.println();
  }
  if ((msg.destinataire & 0x7F) == My_Address)
  {
    if (log_detail>=3) Serial.printf("Message de gateway node:%c\n\r", msg.emetteur);
    if ((msg.emetteur & 0x7F) == SERVER_ADD)
    {
      traitement_message_recu(msg);
    }
  }
}

uint8_t enreg_valeur()
{

  //Serial.printf("Envoi AAA cpt_nb_val:%d\n\r", cpt_nb_val);
  if (!esp_now_actif) return 0;

  if (cpt_nb_val >= NB_VAL_TAB) cpt_nb_val = 0;
  valTemp[cpt_nb_val] = (uint16_t)((Tint+40) * 100);
  valHum[cpt_nb_val] = (uint16_t)(Humid * 100);
  valTick[cpt_nb_val] = Tick_6s; // chaque 6 secondes
  
  cpt_nb_val++;
  uint8_t envoi=0;

  if (cpt_nb_val >= Capt_nb_val_max) envoi=1;
  if (freq_envoi)  // si frequence fixe periodique : envoi si temps total dépassé
  {
    if ((Tick_6s - envoi_6s_prec)*10 >= Capt_tps_total_max) envoi=1;
  }

  if (envoi)
  {
    //Serial.printf("Envoi BBB %d valeurs Temp-HR-Ecart\n\r", cpt_nb_val);
    Message_Struct message;

    message.destinataire = SERVER_ADD | 0x80;  // 0x80 = message hexa
    message.emetteur = My_Address;
    message.statut = 0b01;  // mode veille
    if (My_statut == 3)  message.statut = 0b11; // mode actif
    num_sequentiel++;
    message.num_seq = num_sequentiel;
    message.code = 'C';
    message.code2 = 'T';

    message.payload[0] = cpt_nb_val;
    
    uint8_t pos = 1;
    for (uint8_t cpt = 0; cpt < cpt_nb_val; cpt++)
    {
      message.payload[pos++] = (uint8_t) valTick[cpt];
      message.payload[pos++] = (uint8_t) (valTick[cpt] >> 8);
      message.payload[pos++] = valTick[cpt] >> 16;
      payloadWrite(message.payload, pos, valTemp[cpt]);  
      payloadWrite(message.payload, pos, valHum[cpt]);
      if (log_detail>=4)  Serial.printf("   %.2f°C %.2f%% %d min\n\r", valTemp[cpt]/100.0-40, valHum[cpt]/100.0, valTick[cpt]/10);
    }
    // Taille réelle du message envoyé
    message.longueur = 4 + pos;

    uint8_t result = envoi_data(message, 0);   // envoi périodique des valeurs à la gateway


    if (result == ESP_OK) {
        if (log_detail>=1) Serial.printf("✅ %d valeurs Tick-Temp-HR envoyées\n\r",  message.payload[0]);
        for (uint8_t cpt = 0; cpt < message.payload[0]; cpt++)
        {
          if (log_detail>=1) Serial.printf("Time: %d min   %.2f°C %.2f%%\n\r", valTick[cpt]/10, valTemp[cpt]/100.0-40, valHum[cpt]/100.0);
        }
        cpt_nb_val=0;
        envoi_6s_prec = Tick_6s;  // mettre à jour le temps du dernier envoi
    } else {
        Serial.printf("❌ Erreur envoi ESP-NOW : %d\n\r", result);
        if (log_detail>=4) 
        {
          activation_writelog();
          writeLog('E', 8, valTemp[0]/10,  valHum[0]/10, "Esp_now");
        }
        cpt_nb_val=0;
        envoi_6s_prec = Tick_6s;  // mettre à jour le temps du dernier envoi
        return 1;
    }
    return 0;
  }
  return 0;
}

uint8_t envoi_valeur_instant(float Tint, float Humid, float HA, float vbatt)
{
  if (!esp_now_actif) return 0;

  Message_Struct message;

  message.destinataire = SERVER_ADD | 0x80;  // 0x80 = message hexa
  message.emetteur = My_Address;
  message.statut = 0b01;  // mode veille
  if (My_statut == 3)  message.statut = 0b11; // mode actif
  num_sequentiel++;
  message.num_seq = num_sequentiel;
  message.code = 'C';
  message.code2 = 'I';

  uint8_t pos = 0;
  payloadWrite(message.payload, pos, (uint16_t)((Tint+40) * 100));
  payloadWrite(message.payload, pos, (uint16_t)(Humid * 100));
  payloadWrite(message.payload, pos, (uint16_t)(HA * 100));
  payloadWrite(message.payload, pos, (uint16_t)(vbatt * 100));

  // Taille réelle du message envoyé
  message.longueur = 4 + pos;
  
  if (log_detail>=4) Serial.printf("Message statut avant envoi: %u\n\r", message.statut);

  uint8_t result = envoi_data(message, 0);   // envoi valeurs instantanées à la gateway
  return result;
}


void envoi_temp_hygro()
{
  lecture_Tint(&Tint, &Humid);
  if (log_detail>=2) Serial.printf("milli H2: %lu\n\r", millis());

  float HA = absoluteHumidity(Tint, Humid);
  float vbatt = readBatteryVoltage();
  if (log_detail>=2) Serial.printf("Temp int:%.2f Humid:%.2f HA:%.2f VBatt:%.2f\n\r", Tint, Humid, HA, vbatt); 
  envoi_valeur_instant(Tint, Humid, HA, vbatt);
}
