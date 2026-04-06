
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

//void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len);
//void OnDataRecv(const esp_now_peer_info_t * info, const uint8_t *incomingData, int len);

#if defined(ARDUINO_ARCH_ESP32) && defined(WIFI_TX_INFO_T)
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
#else
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
#endif

uint8_t parseMacString(const char* str, uint8_t mac[6]);



RTC_DATA_ATTR uint8_t mac_gw[6];   // B0:CB:D8:E9:0C:74  adresse mac esp_remote
volatile uint8_t ackReceived = false;  // global pour indiquer que le peer a acké
volatile int ackChannel = -1;       // canal où ça a marché

uint8_t envoi_now(uint8_t channel, esp_now_peer_info_t * peerInfo);


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
float Tint, Text;

uint16_t err_Tint, err_Text, err_Heure;  // compteurs d'erreurs
uint16_t Seuil_batt_sonde;  // millivolt
uint8_t Nb_jours_Batt_log;
uint8_t WIFI_CHANNEL;

RTC_DATA_ATTR uint16_t prolong_veille;
RTC_DATA_ATTR uint8_t action_stockage;
RTC_DATA_ATTR uint8_t action_envoi;

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

// setup : lecture nvs
void setup_nvs()
{

  if (!rtc_valid)  // si le domaine RAM RTC est valide, on ne recharge pas les valeurs de l'eeprom 
  {

    action_stockage = preferences_nvs.getUChar("AcSt", 0);
    if (action_stockage < 2)
      Serial.printf("Action stockage : %i\n\r", action_stockage);
    else {
      action_stockage = 0;
      preferences_nvs.putUChar("AcSt", 0);
      Serial.println("Raz action stockage: 0");
    }

    action_envoi = preferences_nvs.getUChar("AcEn", 0);
    if (action_envoi < 2)         
      Serial.printf("Action envoi : %i\n\r", action_envoi);
    else {
      action_envoi = 0;
      preferences_nvs.putUChar("AcEn", 0);
      Serial.println("Raz action envoi: 0");
    }   


    // periode du cycle : lecture Temp ext par internet
    periode_cycle = preferences_nvs.getUChar("cycle", 0);  // de 10 a 120
    if ((periode_cycle < 10) || (periode_cycle > 120)) {
      periode_cycle = 15;
      preferences_nvs.putUChar("cycle", periode_cycle);
      Serial.printf("Raz periode cycle : val par defaut %imin\n\r", periode_cycle);
    }
    else Serial.printf("periode cycle : %imin\n", periode_cycle);
  }

    #ifdef ESP_VEILLE

      // Initialisation du temps de reveil pour la sonde, si reveil uart/web
      prolong_veille = preferences_nvs.getUShort("PVei", 0);
      if (prolong_veille>=15 && prolong_veille<=600) {
        Serial.printf("Temps reveil : %i sec\n", prolong_veille);
      }
      else
      {  
        prolong_veille = 60;
        preferences_nvs.putUShort("PVei", prolong_veille);
        Serial.printf("Raz temps reveil : %i sec\n", prolong_veille);
      }
    #endif
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
    uint8_t Tint_err = lecture_Tint(&Tint);
    if ((Tint < 1) || (Tint > 45)) {
      Tint = 20.0;
      Tint_err = 7;
    }
    if (Tint_err) log_erreur(Code_erreur_Tint, Tint_err, 1);
    else
      Serial.printf("Temp int:%.2f\n\r", Tint);
}

// apres demarrage reseau
void setup_2()
{
    // Configuration WiFi en mode Station pour ESP-NOW Emetteur

    if ((mode_reseau==13) )
      WiFi.mode(WIFI_STA);
    
    // 🔍 DIAGNOSTIC: Forcer le canal WiFi
    uint8_t current_channel;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&current_channel, &second);
    Serial.printf("Canal WiFi AVANT config ESP-NOW: %d\n", current_channel);
    
    // Forcer le canal si nécessaire (doit correspondre au routeur)
    // esp_wifi_set_promiscuous(true);
    // esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    // esp_wifi_set_promiscuous(false);
    
    if (esp_now_init() != ESP_OK) {
      Serial.println("Erreur initialisation ESP-NOW");
      return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    
    // Vérifier le canal après init
    esp_wifi_get_channel(&current_channel, &second);
    
    Serial.println("\n\n======================================");
    Serial.println("🔵 ESP-NOW Initialisé (RÉCEPTEUR)");
    Serial.print("   MAC Address module: ");
    //if ((mode_reseau==13) )
    //else
    //  Serial.println(WiFi.softAPmacAddress());
    Serial.println(WiFi.macAddress());

    // Stockage de l'adresse MAC dans le tableau mac_gw[6]
    Serial.printf("   MAC dest : %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_gw[0], mac_gw[1], mac_gw[2],
            mac_gw[3], mac_gw[4], mac_gw[5]);

    Serial.printf("   Canal WiFi: %d\n", current_channel);
    Serial.println("   En attente de messages...");
    Serial.println("======================================\n\n");
    delay(2000); // 2 secondes de pause pour lire
}



void appli_event_on(systeme_eve_t evt)
{
}

void appli_event_off(systeme_eve_t evt)
{
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
  uint8_t res=1;


  if (reg == 9)  // registre 9 : Seuil batterie sonde
  {
    res = 0;
    *valeur = Seuil_batt_sonde;
  }
  if (reg == 10)  // registre 10 : Nb de jours Log batterie
  {
    res = 0;
    *valeur = Nb_jours_Batt_log;
  }
  if (reg == 16)  // registre 16 : duree allumage
  {
    res = 0;
    *valeur = prolong_veille;
  }
  if (reg == 17)  // registre 17 : action stockage
  {
    res = 0;
    *valeur = action_stockage;
  }
  if (reg == 18)  // registre 18 : action envoi
  {
    res = 0;
    *valeur = action_envoi;
  }
  if (reg == 41)  // registre 41 : canal WiFi actuel
  {
    res = 0;
    uint8_t current_channel;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&current_channel, &second);
    *valeur = (float)current_channel;
  }
  if (reg == 42)  // registre 42 : canal WiFi preferentiel
  {
    res = 0;
    *valeur = WIFI_CHANNEL;
  }

  return res;
}

// type 2
uint8_t requete_SetReg_appli(int param, float valeurf)
{
  int16_t valeur = int16_t(round(valeurf));
  uint8_t res = 1;

  if (param == 9)  // registre 9 : Seuil batterie sonde
  {
    if ((valeur >=1800 ) && (valeur <= 4500)) {
      res = 0;
      Seuil_batt_sonde = valeur;
      preferences_nvs.putUShort("SeBa", Seuil_batt_sonde);
    }
  }


  if (param == 10)  // registre 10 : Nb jours log batterie
  {
    if ((valeur) && (valeur < 16)) {
      res = 0;
      Nb_jours_Batt_log = valeur;
      preferences_nvs.putUChar("FrBL", Nb_jours_Batt_log);
    }
  }
  if (param == 16)  // registre 16 : duree allumage
  {
    if ((valeur>=15) && (valeur <= 600)) {  // de 15 sec à 10 minutes
      res = 0;
      prolong_veille = valeur;
      preferences_nvs.putUShort("PVei", prolong_veille);
    }
  }
  if (param == 17)  // registre 17 : action stockage
  {    if ((valeur == 0) || (valeur == 1))
    {      res = 0;
      action_stockage = valeur;
      preferences_nvs.putUChar("AcSt", action_stockage);
    }
  }
  if (param == 18)  // registre 18 : action envoi      
  {    if ((valeur == 0) || (valeur == 1))
    {      res = 0;
      action_envoi = valeur;
      preferences_nvs.putUChar("AcEn", action_envoi);
    }
  }

  if (param == 41)  // registre 41 : last_wifi_channel
  {
    if ((valeur) && (valeur <= 13))
    {
      res = 0;
      last_wifi_channel = valeur;
    }
  }
  if (param == 42)  // registre 42 : canal wifi preferentiel
  {
    if ((valeur) && (valeur <= 13))
    {
      res = 0;
      WIFI_CHANNEL = valeur;
      preferences_nvs.putUChar("WifiC", WIFI_CHANNEL);
    }
  }

  return res;
}

// type 4
uint8_t requete_Get_String_appli(uint8_t type, String var, char *valeur)
{
  uint8_t res=1;
  int paramV = var.toInt();
  // valeur limité a 50 caractères
  
  if (paramV == 11)  // registre 11 : adresse MAC ce module
  {
    res = 0;
    strncpy(valeur, WiFi.macAddress().c_str(), 18);
  }
  if (paramV == 12)  // registre 12 : adresse MAC destinataire
  {
    res = 0;
    snprintf(valeur, 18,
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_gw[0], mac_gw[1], mac_gw[2],
           mac_gw[3], mac_gw[4], mac_gw[5]);
  }

  return res;
}

uint8_t parseMacString(const char* str, uint8_t mac[6]) {
  int v[6];
  if (sscanf(str, "%x:%x:%x:%x:%x:%x",
             &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
    return false;
  }
  for (int i = 0; i < 6; i++) mac[i] = (uint8_t)v[i];
  return true;
}

// type 4
uint8_t requete_Set_String_appli(int param, const char *texte)
{
  uint8_t res=1;
  IPAddress ip;

    if (param == 12)  // registre 12 : adresse Mac dest
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

  if (strcmp(reg, "Tint") == 0) 
    { 
      res=0; 
      uint8_t Tint_erreur = lecture_Tint(&Tint);
      Serial.println(Tint_erreur);
      Serial.println(Tint);
    }

  if (strcmp(reg, "EncP") == 0) 
  { 
      res=0; 
      encodeP();
      Serial.println("encodage ok");
  }
    
  return res;
}


// erreur :0:ok  sinon erreur 2 à 7
uint8_t lecture_Tint(float *mesure)
{
  uint8_t Tint_erreur = 7;
  float valeur = 20;

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
  Serial.printf("lecture Tint : %.2f Err:%i\n\r", valeur, Tint_erreur);
  *mesure = valeur;
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
  sprintf(url, "http://api.open-meteo.com/v1/forecast?latitude=%s&longitude=%s&current=temperature_2m", LATITUDE, LONGITUDE);


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
    cout_moy24h += 0;  // 1000
  }
  cpt24_Cout++;

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

// Callback reception ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
//void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
//void OnDataRecv(const esp_now_peer_info_t * info, const uint8_t *incomingData, int len) {
  // 🔍 DIAGNOSTIC: Afficher infos de réception
  Serial.println("\n📥 ========== RECEPTION ESP-NOW ==========");
  for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < 5) Serial.print(":");
    }
  /*Serial.printf("   Source MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                info->src_addr[0], info->src_addr[1], info->src_addr[2],
                info->src_addr[3], info->src_addr[4], info->src_addr[5]);*/
  
  // Afficher le canal WiFi actuel
  uint8_t current_channel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&current_channel, &second);
  Serial.printf("   Canal WiFi actuel: %d\n", current_channel);
  Serial.printf("   Taille reçue: %d octets\n", len);
  
  Message_EspNow receivedMessage;
  memcpy(&receivedMessage, incomingData, sizeof(receivedMessage));

  Serial.print("   Type: "); Serial.print(receivedMessage.type);
  Serial.print(" | Valeur: "); Serial.println(receivedMessage.value);
  Serial.println("=========================================\n");

  if (receivedMessage.type == 1) { // Temperature
    float Trecue = receivedMessage.value;
    if ((Trecue > -10.0) && (Trecue < 49.99f)) 
    {
      Tint = Trecue;
      if ((Trecue < 24.99) || (Trecue > 25.01))
      {
        unsigned long mil = millis();
        if (mil - last_remote_Tint_time > 25*60*1000) // le precedent message est vieux de plus de 25 minutes
          err_Tint++;
        last_remote_Tint_time = mil;
        cpt24_Tint++;
        tempI_moy24h += Tint;
      }
    }
    Serial.printf("✅ Tint mise à jour: %.2f°C\n", Tint);
  }
  else if (receivedMessage.type == 2) { // Batterie
    Vbatt_Th = receivedMessage.value;
    Serial.printf("✅ Vbatt_Th mise à jour: %.2fV\n", Vbatt_Th);
    Vbatt_Th_I = 1;
  }
  else {
    Serial.printf("⚠️ Type de message inconnu: %d\n", receivedMessage.type);
  }
}



// Sonde de temperature envoie valeur de temp à la chaudiere par ESP_now
void envoi_temp_esp_chaudiere()
{
    // --- MODE THERMOMETRE DISTANT (ESP-NOW) ---
    uint8_t Tint_erreur = lecture_Tint(&Tint);  // Mesure locale
    if (Tint_erreur) 
    {
      Tint=25.0;
      #ifdef DEBUG
            Tint = 18.0;
      #endif
    }

    if (mac_gw[0] || mac_gw[3] || mac_gw[4])
    {
      // Initialisation WiFi en mode Station (nécessaire pour ESP-NOW)
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();

      if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        ESP.restart();
      }

      esp_now_register_send_cb(OnDataSent);

      // Préparation du Peer (Chaudière)
      esp_now_peer_info_t peerInfo;
      memset(&peerInfo, 0, sizeof(peerInfo)); // Initialisation complète à zéro
      memcpy(peerInfo.peer_addr, mac_gw, 6);
      peerInfo.channel = 0; // Le canal sera défini avant l'ajout
      peerInfo.encrypt = false;
      peerInfo.ifidx = WIFI_IF_STA; // Interface WiFi Station (OBLIGATOIRE)

      // 🚀 OPTION 1 : Forcer le canal connu (plus rapide et économe en énergie)
      // Si vous connaissez le canal de votre routeur, décommentez ces lignes :
      /*
      Serial.printf("🎯 Forçage canal %d (défini dans variables.h)\n", WIFI_CHANNEL);
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);
      last_wifi_channel = WIFI_CHANNEL; // Pour la prochaine fois
      */

      // 🔍 OPTION 2 : Scan robuste des canaux (si le canal n'est pas connu ou change)

      Serial.printf("🔍 Scan de 13 canaux (priorité: canal %d)\n", last_wifi_channel);
      uint8_t deliverySuccess = false;
      uint8_t current_channel;
      if (!last_wifi_channel || last_wifi_channel>13) last_wifi_channel=1;  // si corrompu : channel 1

      if (etat_now==0)  // encore aucun envoi essayé
      {
        for (uint8_t k = 0; k < 13; k++)  // => channel 1 à 13
        {
          current_channel = k + last_wifi_channel;
          if (current_channel > 13) current_channel -= 13;
          deliverySuccess = envoi_now(current_channel, &peerInfo);
          if (deliverySuccess) break;
        }
        if (deliverySuccess) etat_now=2;
        else etat_now=1;
      }
      else if (etat_now==2)  // essai precedent reussi
      {
        deliverySuccess = envoi_now(last_wifi_channel, &peerInfo);
        if (!deliverySuccess) etat_now=4; // prochain essai sur le meme channel+ scan
      }
      else if (etat_now==1 || etat_now==3)  // essai precedent raté
      {
        deliverySuccess = envoi_now(last_wifi_channel, &peerInfo);
        if (deliverySuccess) etat_now=2; 
        else 
        {
          // prochain essai sur le channel suivant
          last_wifi_channel++;
          if (last_wifi_channel > 13) last_wifi_channel=1;
        }
      }
      else if (etat_now==4)  // envoi precedent raté mais celui d'avan réussi
      {
        for (uint8_t k = 0; k < 14; k++)  // => channel 1 à 13 et 1 de plus
        {
          current_channel = k + last_wifi_channel;
          if (current_channel > 13) current_channel -= 13;
          deliverySuccess = envoi_now(current_channel, &peerInfo);
          if (deliverySuccess) break;
        }
        if (deliverySuccess) etat_now=2;
        else etat_now=3;
      }
      else etat_now=0; // si etat_now corrompu, remise à 0

      Serial.printf("etat_now:%i\n\r", etat_now);
      
      cpt_cycle_batt++;
      if (cpt_cycle_batt >= 90) cpt_cycle_batt=0;

      if (deliverySuccess) // envoi reussi
      {   
        // Envoi tension batterie tous les 90 cycles (23h)
        if (!cpt_cycle_batt)
        {
          float Vbatt = readBatteryVoltage();
          delay(50);
          Message_EspNow message;
          message.type = 2; // Batterie
          message.value = Vbatt;
          esp_now_send(mac_gw, (uint8_t *) &message, sizeof(message));
          Serial.printf("Envoi batterie: %.2fV (cycle %d)\n", Vbatt, cpt_cycle_batt);
        }
      }
    }
    else
      Serial.println("Adresse Mac chaudiere nulle");

    // Deep Sleep

      uint64_t sleep_time = (uint64_t)periode_cycle * 60 * 1000000;
      if (mode_rapide==12)
      sleep_time = (uint64_t)periode_cycle * 1000000;
      passage_deep_sleep(sleep_time);
}

uint8_t envoi_now(uint8_t channel, esp_now_peer_info_t * peerInfo)
{
  uint8_t result = false;

  // Fixer le canal
  Serial.printf("\n--- Essai canal %d ---\n", channel);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  
  // Vérifier que le canal a bien été changé
  uint8_t actual_channel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&actual_channel, &second);
  
  if (actual_channel != channel)
  {
    Serial.printf("⚠️ Échec changement canal (demandé:%d, actuel:%d)\n", channel, actual_channel);
    delay(100); // Attendre un peu plus
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_get_channel(&actual_channel, &second);
    Serial.printf("   2ème tentative: canal actuel=%d\n", actual_channel);
  } else {
    //Serial.printf("✅ Canal changé: %d\n", actual_channel);
  }
  
  delay(50); // Délai pour stabilisation du canal

  // Ajouter le peer sur ce canal
  if (esp_now_is_peer_exist(mac_gw)) {
    esp_now_del_peer(mac_gw);
  }
  peerInfo->channel = actual_channel; // Utiliser le canal réel
  if (esp_now_add_peer(peerInfo) != ESP_OK){
    Serial.println("❌ Échec ajout peer");
  }
  //Serial.println("✅ Peer ajouté");

  // Envoi Température
  Message_EspNow message;
  message.type = 1; // Température
  message.value = Tint;
  
  // 🔍 DIAGNOSTIC: Afficher les infos avant envoi
  /*Serial.printf("📤 Tentative envoi sur canal %d vers MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                actual_channel,
                mac_gw[0], mac_gw[1], mac_gw[2],
                mac_gw[3], mac_gw[4], mac_gw[5]);*/
  Serial.printf("   Message: Type=%d, Valeur=%.2f°C\n", message.type, message.value);
  
  ackReceived=0;
  ackChannel = -1;
  esp_err_t resulta = esp_now_send(mac_gw, (uint8_t *) &message, sizeof(message));

  if (resulta == ESP_OK)
  {
    //Serial.printf("Envoye sur canal %d\n", actual_channel);

    // attendre la réponse max 100 ms
    int wait = 0;
    while (!ackReceived && wait < 10) { // 10 * 10ms = 100ms
        delay(10);
        wait++;
    }

    if (ackReceived) // canal trouvé
    {
      result = true; 
      Serial.println("✅ Ack Recu");
      if (last_wifi_channel != actual_channel)
      {
        last_wifi_channel = actual_channel;
      }
    }
  }
  else Serial.println("❌ Echec d'envoi");

  return result;
}

// prise de video , enreg SDcard, compression petite image et envoi
void enreg_video()
{

}

void prise_video()
{}
void prise_photo()
{}