#include <Arduino.h>
#include "ThingSpeak.h"
#include <WiFi.h>
#include "config.h"
#include <HTTPClient.h>
#include "ThingSpeakData.h"
#include <string>

#include <Preferences.h>

WiFiClient client;
//Config con;

namespace THINGSPEAK {
//Þessar breytur verða allar overwritten seinna
int channelNumber = 2596560;
String writeApi = "ON9DWVHJRVRZG02B";
int serial_number = 0;

//Nafn og password hjá wifi-pung
const char* ssid = "HUAWEI_E5785_0B6F";           
const char* password = "Cemis2024";


void SetupWiFi(float warmup) {
  // Byrjar tengingu við netið
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Tengist við Wi-Fi");

  //Laga þetta "k" ógeð
  //Notað til þess að hann hætti að reyna að tengjast við wifi eftir einhvern tíma
  //Þarf að breyta því nú reynir hann ekki að tengjast wifi strax í warmup heldur þegar 75% af pre'inu er lokið
  int k = 0;
  while (WiFi.status() != WL_CONNECTED && k < warmup) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.print(".");
    k++;
  }


  //Segir á Serial Monitor hvort að tenging hafi heppnast
  if (WiFi.status() == WL_CONNECTED) {
  // Notify when connected
  Serial.println("");
  Serial.print("Tengt við WiFi: " + String(WiFi.localIP()));
  } else {
    Serial.println("");
    Serial.println("Náði ekki tengingu við WiFi");
  }
}

//Setur inn status inn á thingspeak
void Status(String STATUS) { ThingSpeak.setStatus(STATUS); }


//Notar Prefrances til þess að ná í serial_number úr non-volitile minni (NVM).
String getSerialNumber(){
  Preferences pref;
  pref.begin("chamberconf");
  String serial_number = pref.getString("serial_number", "default_serial_number");
  pref.end();
  return serial_number;
}

//Setup fyrir ThingSpeak aðgerðir. 
void setup_ThingSpeak(){
  ThingSpeak.begin(client);
  //Nær í serial_number úr getSerialNumber og breytir í integer
  serial_number = getSerialNumber().toInt();

  //Fylki sem inniheldur öll API-write keys
  String APIFylki[11] = {"P73S7SK8EDZVDEKG", "PU83AXC9WS82485R", "BPY4Y95TS9V78IU2", "X3WI8I7LFEG1870V", "JO6MKDXVO1Y2D8XC", 
  "JU7MXCBJRYLTY6XI", "89JDLKO3XWHNMG4I", "ON9DWVHJRVRZG02B", "7WVX49VGICKTH9U5", "7448RZ4PG5CFZWCU", "09QIGOXAD1Q2B8YA"};

  //Fylki sem inniheldur channel-number's
  int channelFylki[11] = {2607801, 2607803, 2607804, 2607806, 2607807, 2607808, 2607809, 2596560, 2601407, 2601433, 2604966};

  //Gefur tækinu API-write-key og Channel Number
  writeApi = APIFylki[serial_number];
  channelNumber = channelFylki[serial_number]; //channelFylki[serial_number];
}

//Fall sem segir okkur hvort tækið sé tengd við netið
bool isConnected(){
    return WiFi.status() == WL_CONNECTED;
}

void WriteAll(float co2, float raki, float hiti, float thryst, bool VALVES_CLOSED){
  Serial.println("Tengist við ThingSpeak");
  if (isConnected() && client.connect(THINGSPEAK_URL, 80)){
    ThingSpeak.setField(1,co2);
    ThingSpeak.setField(2, raki);
    ThingSpeak.setField(3, hiti);
    ThingSpeak.setField(4, thryst);

    //Mögulega henda þessu
    //Status(VALVES_CLOSED);
    if (VALVES_CLOSED){
      ThingSpeak.setField(5,"1");
    }
    else{
      ThingSpeak.setField(5,"0");
    }

    //Fyrir debug
    Serial.print("Serial Number er: ");
    Serial.println(serial_number);
    Serial.print("API-Key er: ");
    Serial.println(writeApi);
    Serial.print("Channel er: ");
    Serial.println(channelNumber);


    int x = ThingSpeak.writeFields(channelNumber, writeApi.c_str());
    if(x == 200){
      Serial.println("Tókst að senda gögn");
      //return; // fara úr falli ef það tekst að senda gögn.
    } else {
      Serial.println("Tókst ekki að senda gögn");
      }

  } else {
    Serial.println("Tókst ekki ad tengjast við ThingSpeak");
    //return;
    }
  }


//Eftirfarandi föll eru eingungis til þess að getað sett eitt field í einu, getur verið þæginlegt til að debugga eða nota ef þörf er á
void Koltvioxid(int channelNumber, float co2){
  if (isConnected() && client.connect(THINGSPEAK_URL,80)) {
    int x = ThingSpeak.writeField(channelNumber, 1, co2, writeApi.c_str());
    Serial.println("X: " + String(x));
  } else {
    Serial.print("Nær ekki tengingu til að senda koltvíoxíð.");
  }
}

void Raki(int channelNumber, float raki){
  if (isConnected() && client.connect(THINGSPEAK_URL,80)) {
    int x =  ThingSpeak.writeField(channelNumber, 2, raki, writeApi.c_str());
    Serial.println("X: " + String(x));
  } 
  else {
    Serial.print("Nær ekki tengingu til að senda raka");
  }
}

void Hitastig(int channelNumber, float hiti){
    if (isConnected() && client.connect(THINGSPEAK_URL,80)){
      int x = ThingSpeak.writeField(channelNumber, 3, hiti, writeApi.c_str());
      Serial.println("X: " + String(x));
    }
    else{
      Serial.print("Nær ekki tengingu til að senda hitastig.");
    }
}

void Thristingur(int channelNumber, float thrist){
    if (isConnected() && client.connect(THINGSPEAK_URL,80)){
      int x = ThingSpeak.writeField(channelNumber, 4, thrist, writeApi.c_str());
      Serial.println("X: " + String(x));
    }
    else{
      Serial.print("Nær ekki tengingu til að senda þrýsting.");
    }
}
}
