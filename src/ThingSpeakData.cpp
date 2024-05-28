#include <Arduino.h>
#include "ThingSpeak.h"
#include <WiFi.h>
#include "config.h"
#include <HTTPClient.h>
#include "ThingSpeakData.h"
#include <string>

WiFiClient client;

namespace THINGSPEAK {
float channelNumber = 2546046;
String writeApi = "";
const char* ssid = "AtDiddys";           
const char* password = "diddi2389";
int serial_number = 0;

void SetupWiFi() {
  // Byrjar tengingu við netið
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Tengist við Wi-Fi");

  int k = 0;
  while (WiFi.status() != WL_CONNECTED && k < 3600) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
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

  //Fylki sem inniheldur öll API-write keys og Channel Number
  String APIFylki[7] = {"I9IJWNMPNF2TKY12", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L"};
  float channelFylki[7] = {2546046, 2548253, 2548253, 2548253, 2548253, 2548253, 2548253};

  //Gefur tækinu API-write-key og Channel Number
  writeApi = APIFylki[serial_number];
  channelNumber = channelFylki[serial_number];
}

//Fall sem segir okkur hvort tölvan er tengd við netið
bool isConnected(){
    return WiFi.status() == WL_CONNECTED;
}

void WriteAll(float co2, float raki, float hiti, float thryst){
  if (isConnected() && client.connect(THINGSPEAK_URL, 80)){
    Serial.println("Tengist við ThingSpeak");
    ThingSpeak.setField(1,co2);
    ThingSpeak.setField(2, raki);
    ThingSpeak.setField(3, hiti);
    ThingSpeak.setField(4, thryst);


    Serial.print("Serial Number er: ");
    Serial.println(serial_number);
    Serial.print("API-Key er: ");
    Serial.println(writeApi);



    int x = ThingSpeak.writeFields(channelNumber, writeApi.c_str());
    if(x == 200){
      Serial.println("Tókst að senda gögn");
    }
    else{
      Serial.println("Tókst ekki að senda gögn");
    }
  }
  else{
    Serial.println("Tókst ekki að tengjast við ThingSpeak");
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
