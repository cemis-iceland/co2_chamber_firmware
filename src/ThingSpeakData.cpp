#include <Arduino.h>
#include "ThingSpeak.h"
#include <WiFi.h>
#include "config.h"
#include <HTTPClient.h>
#include "ThingSpeakData.h"
#include <string>

//WriteAPI = I9IJWNMPNF2TKY12
//ReadAPI = BI3IRYRDYRCIO4EO
WiFiClient client;

namespace THINGSPEAK {
float channelNumber = 2546046;
const char* writeApi = "I9IJWNMPNF2TKY12";
const char* ssid = "nei";           
const char* password = "mammathin01";

void SetupWiFi() {
  // Byrjar tengingu við netið5
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
  // kallar á "SetupWifi" og tengist við netið

  // String channelsFylki[2][7] = {{"I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12",
  // "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12",
  // "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12"},{"2546046", "2546046", "2546046",
  // "2546046", "2546046", "2546046", "2546046"}}; //Fylki með öllum channels.
  // Seinni strengurinn er read kóði en nota hann tímabundið

  // writeApi = channelsFylki[serial_number];

  // String channel = channelsFylki[serialNr];

  // if (serial_number == 1.1){writeApi = "I9IJWNMPNF2TKY12";}
  // else if (serial_number == 1.2){writeApi = "I9IJWNMPNF2TKY12";}
  // else if (serial_number == 2.1){writeApi = "I9IJWNMPNF2TKY12";}
  // else if (serial_number == 2.2){writeApi = "I9IJWNMPNF2TKY12";}
  // else if (serial_number == 3.1){writeApi = "I9IJWNMPNF2TKY12";}
  // else if (serial_number == 3.2){writeApi = "I9IJWNMPNF2TKY12";}
  // else if (serial_number == 4.1){writeApi = "I9IJWNMPNF2TKY12";}
  // else {printf("Serial number not found");}

  // thingURL = thingURL + String(writeApi);

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
