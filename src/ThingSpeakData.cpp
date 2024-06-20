#include <Arduino.h>
#include "ThingSpeak.h"
#include <WiFi.h>
#include "config.h"
#include <HTTPClient.h>
#include "ThingSpeakData.h"
#include <string>
#include <Preferences.h>

WiFiClient client;

namespace THINGSPEAK {
int channelNumber = 2546046;
String writeApi = "";
int serial_number = 0;

const char* ssid = "HUAWEI_E5785_0B6F";           
const char* password = "Cemis2024";


void SetupWiFi(float warmup) {
  // Byrjar tengingu við netið
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Tengist við Wi-Fi");

  int k = 0;
  while (WiFi.status() != WL_CONNECTED && k < warmup) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.print(".");
    k++;
  }


  if (WiFi.status() == WL_CONNECTED) {
    // Notify when connected
  Serial.println("");
  Serial.print("Tengt við WiFi: " + String(WiFi.localIP()));
  } else {
    Serial.println("");
    Serial.println("Failed to connect to WiFi");
  }

}

//Setup fyrir ThingSpeak aðgerðir. 
void setup_ThingSpeak(int serialNR){
  ThingSpeak.begin(client);
  serial_number = serialNR;
  Serial.println("Setur upp thingspeak.");

  //Fylki sem inniheldur öll API-write keys og Channel Number
  String APIFylki[7] = {"FE7T0FTS1L2BDTY4", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L"};

  //Seinna meir hægt að bæta við read API ef þess þarf

  int channelFylki[7] = {2574868, 2548253, 2548253, 2548253, 2548253, 2548253, 2548253};

  //Gefur tækinu API-write-key og Channel Number
  writeApi = APIFylki[serial_number];
  channelNumber = channelFylki[serial_number];

  //Nær í staðsetningu tækis frá prefrances
  Preferences pref;
  float longditude = pref.getFloat("longditude", longditude);
  float latitude = pref.getFloat("latitude", latitude);

  //Setur upp staðsetningu á tækinu inná thingspeak, bara einu sinni.
  ThingSpeak.setLongitude(longditude);
  ThingSpeak.setLatitude(latitude);
}

//Fall sem segir okkur hvort tölvan er tengd við netið
bool isConnected(){
    return WiFi.status() == WL_CONNECTED;
}

void WriteAll(float co2, float raki, float hiti, float thryst, bool VALVES_CLOSED){
  //int maxTries=60;
  //int retry = 0;
  // Skrifa gögn upp á thingspeak, gera aðra tilraun ef nettenging næst ekki eða það tekst ekki að senda gögn.
  // max 5x til þess að koma í veg fyrir yfirflæði frá þessu falli.
  //while(retry < maxTries) {
  Serial.println("Tengist við ThingSpeak");
  if (isConnected() && client.connect(THINGSPEAK_URL, 80)){
    ThingSpeak.setField(1,co2);
    ThingSpeak.setField(2, raki);
    ThingSpeak.setField(3, hiti);
    ThingSpeak.setField(4, thryst);

    //Setur status inná skjalinu sem fæst frá thingspeak.com
    if(VALVES_CLOSED){
      ThingSpeak.setStatus("Valves Closed");
    }
    else{
      ThingSpeak.setStatus("Valves Open");
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
    return;
    //Serial.println("Endurræsi nettengingu.");
    }
  }


//Eftirfarandi föll eru eingungis til þess að getað sett eitt field í einu, getur verið þæginlegt til að debugga
void Koltvioxid(float co2){
  if (isConnected() && client.connect(THINGSPEAK_URL,80)) {
    int x = ThingSpeak.writeField(2546046, 1, co2, writeApi.c_str());
    Serial.println("X: " + String(x));
  } else {
    Serial.print("Nær ekki tengingu til að senda koltvíoxíð.");
  }
}

void Raki(float raki){
  if (isConnected() && client.connect(THINGSPEAK_URL,80)) {
    int x =  ThingSpeak.writeField(2546046, 2, raki, writeApi.c_str());
    Serial.println("X: " + String(x));
  } 
  else {
    Serial.print("Nær ekki tengingu til að senda raka");
  }
}

void Hitastig(float hiti){
    if (isConnected() && client.connect(THINGSPEAK_URL,80)){
      int x = ThingSpeak.writeField(2546046, 3, hiti, writeApi.c_str());
      Serial.println("X: " + String(x));
    }
    else{
      Serial.print("Nær ekki tengingu til að senda hitastig.");
    }
}

void Thristingur(float thrist){
    if (isConnected() && client.connect(THINGSPEAK_URL,80)){
      int x = ThingSpeak.writeField(2546046, 4, thrist, writeApi.c_str());
      Serial.println("X: " + String(x));
    }
    else{
      Serial.print("Nær ekki tengingu til að senda þrýsting.");
    }
}

}
