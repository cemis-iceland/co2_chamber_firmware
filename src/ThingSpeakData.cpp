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
const char* writeApi = "I9IJWNMPNF2TKY12";
const char* ssid = "AtDiddys";           
const char* password = "diddi2389";

void SetupWiFi() {
  // Byrjar tengingu við netið
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  // Serial.print("Tengist við Wi-Fi")
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
  }

  // Lætur vita þegar það nær tengingu
  Serial.println("");
  Serial.print("Tengt við WiFi: " + WiFi.localIP());
}

//Setup fyrir ThingSpeak aðgerðir. 
void setup_ThingSpeak(int serial_number){
  ThingSpeak.begin(client);

  //Fylki sem inniheldur öll API-write keys og Channel Number
  //String APIFylki[7] = {"I9IJWNMPNF2TKY12", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L", "H1SDZ72WOUCP8K0L"};
  //float channelFylki[7] = {2546046, 2548253, 2548253, 2548253, 2548253, 2548253, 2548253};

  //Gefur tækinu API-write-key og Channel Number
  //writeApi = APIFylki[serial_number];
  //channelNumber = channelFylki[serial_number];

  //Serial.print("writeApi: ");
  //Serial.println(writeApi);
  //Serial.print("channelNumber: ");
  //Serial.println(channelNumber);
  //channelNumber = 2546046;
  //String writeApi = "I9IJWNMPNF2TKY12";

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
    int x = ThingSpeak.writeFields(channelNumber, writeApi);
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
    int x = ThingSpeak.writeField(2546046, 1, co2, writeApi);
    Serial.println("X: " + String(x));
  } else {
    Serial.print("Nær ekki tengingu til að senda koltvíoxíð.");
  }
}

void Raki(float raki){
  if (isConnected() && client.connect(THINGSPEAK_URL,80)) {
    int x =  ThingSpeak.writeField(2546046, 2, raki, writeApi);
    Serial.println("X: " + String(x));
  } 
  else {
    Serial.print("Nær ekki tengingu til að senda raka");
  }
}

void Hitastig(float hiti){
    if (isConnected() && client.connect(THINGSPEAK_URL,80)){
      int x = ThingSpeak.writeField(2546046, 3, hiti, writeApi);
      Serial.println("X: " + String(x));
    }
    else{
      Serial.print("Nær ekki tengingu til að senda hitastig.");
    }
}

void Thristingur(float thrist){
    if (isConnected() && client.connect(THINGSPEAK_URL,80)){
      int x = ThingSpeak.writeField(2546046, 3, thrist, writeApi);
      Serial.println("X: " + String(x));
    }
    else{
      Serial.print("Nær ekki tengingu til að senda þrýsting.");
    }
}

}
