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
const char* writeApi = "I9IJWNMPNF2TKY12";
const char* ssid = "AtDiddys";           
const char* password = "diddi2389";
void SetupWiFi(const char* ssid, const char* password) {
  // Byrjr tengingu við netið
  WiFi.mode(WIFI_MODE_STA);
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
    //kallar á "SetupWifi" og tengist við netið

    //String channelsFylki[2][7] = {{"I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12"},{"2546046", "2546046", "2546046", "2546046", "2546046", "2546046", "2546046"}}; //Fylki með öllum channels. Seinni strengurinn er read kóði en nota hann tímabundið

    //writeApi = channelsFylki[serial_number];

    //String channel = channelsFylki[serialNr];

    // if (serial_number == 1.1){writeApi = "I9IJWNMPNF2TKY12";}
    // else if (serial_number == 1.2){writeApi = "I9IJWNMPNF2TKY12";}
    // else if (serial_number == 2.1){writeApi = "I9IJWNMPNF2TKY12";}
    // else if (serial_number == 2.2){writeApi = "I9IJWNMPNF2TKY12";}
    // else if (serial_number == 3.1){writeApi = "I9IJWNMPNF2TKY12";}
    // else if (serial_number == 3.2){writeApi = "I9IJWNMPNF2TKY12";}
    // else if (serial_number == 4.1){writeApi = "I9IJWNMPNF2TKY12";}
    // else {printf("Serial number not found");}

    //thingURL = thingURL + String(writeApi);

}

//Fall sem segir okkur hvort tölvan er tengd við netið
bool isConnected(){
    return WiFi.status() == WL_CONNECTED;
}


void Koltvioxid(float co2){
  SetupWiFi(ssid, password);
  if (isConnected()) {
    int x = ThingSpeak.writeField(long(2546046), 1, co2, writeApi);
  } else {
    Serial.print("Nær ekki tengingu til að senda koltvíoxíð");
  }
}

void Raki(float raki){
  SetupWiFi(ssid, password);
  if (isConnected()) {
    int x =  ThingSpeak.writeField(2546046, 2, raki, writeApi);
    Serial.println("X: " + String(x));
  } else {
    Serial.print("Nær ekki tengingu til að senda raka");
  }
}

void Hitastig(float hiti){
    SetupWiFi(ssid, password);
    if (isConnected()){
        ThingSpeak.writeField(2546046, 3, hiti, writeApi);
    }
    else{
        Serial.print("Nær ekki tengingu til að senda hitastig");
    }
}

void Thristingur(float thrist){
    SetupWiFi(ssid, password);
    if (isConnected()){
        ThingSpeak.writeField(2546046, 3, thrist, writeApi);
    }
    else{
        Serial.print("Nær ekki tengingu til að senda þrýstingu");
    }
}

}
