#include <Arduino.h>
#include "ThingSpeak.h"
#include <WiFi.h>
#include "config.h"
#include <HTTPClient.h>
#include "ThingSpeakData.h"
#include <string>

WiFiClient client;

namespace THINGSPEAK {
int channelNumber = 2596560;
String writeApi = "ON9DWVHJRVRZG02B";
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

void Status(String STATUS) { ThingSpeak.setStatus(STATUS); }

//Setup fyrir ThingSpeak aðgerðir. 
void setup_ThingSpeak(int serialNR){
  ThingSpeak.begin(client);
  serial_number = serialNR;
  Serial.println("Setur upp thingspeak.");

  //Fylki sem inniheldur öll API-write keys og Channel Number
  String APIFylki[7] = {"ON9DWVHJRVRZG02B", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "ON9DWVHJRVRZG02B", "ON9DWVHJRVRZG02B"};

  //Seinna meir hægt að bæta við read API ef þess þarf

  int channelFylki[7] = {2596560, 2548253, 2548253, 2548253, 2548253, 2596560, 2596560};

  //Gefur tækinu API-write-key og Channel Number
  writeApi = APIFylki[serial_number];
  channelNumber = channelFylki[serial_number]; //channelFylki[serial_number];
}

//Fall sem segir okkur hvort tölvan er tengd við netið
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

    //Status(VALVES_CLOSED);
    if (VALVES_CLOSED){
      ThingSpeak.setField(5,"1");
    }
    else{
      ThingSpeak.setField(5,"0");
    }

    //Setur status inná skjalinu sem fæst frá thingspeak.com


    //Fyrir debug
    Serial.print("Serial Number er: ");
    Serial.println(serial_number);
    Serial.print("API-Key er: ");
    Serial.println(writeApi);
    Serial.print("Channel er: ");
    Serial.println(channelNumber);
    if (VALVES_CLOSED){
     ThingSpeak.setStatus("1");
    }
    else{
     ThingSpeak.setStatus("0");
    }

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

void Status(bool VALVES_CLOSED){
  if(VALVES_CLOSED){
    ThingSpeak.setStatus("1");
  }
  else{
    ThingSpeak.setStatus("0");
  }
}
}
