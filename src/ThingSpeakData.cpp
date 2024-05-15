#include <Arduino.h>
#include "ThingSpeak.h"
#include <WiFi.h>
#include "config.h"
#include <HTTPClient.h>
#include "ThingSpeakData.h"

//WriteAPI = I9IJWNMPNF2TKY12
//ReadAPI = BI3IRYRDYRCIO4EO

namespace THINGSPEAK {
const char* writeApi;
const char* ssid = "nei";           
const char* password = "mammathin01";
void SetupWiFi(const char* ssid, const char* password) {
  // Byrjr tengingu við netið
  WiFi.begin(ssid, password);
  // Serial.print("Tengist við Wi-Fi")
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Lætur vita þegar það nær tengingu
  Serial.println("");
  Serial.print("Tengt við WiFi: " + WiFi.localIP());
}

//Setup fyrir ThingSpeak aðgerðir. 
void setup_ThingSpeak(int serial_number){
    //kallar á "SetupWifi" og tengist við netið
    SetupWiFi(ssid, password);

    //String channelsFylki[2][7] = {{"I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12", "I9IJWNMPNF2TKY12"},{"2546046", "2546046", "2546046", "2546046", "2546046", "2546046", "2546046"}}; //Fylki með öllum channels. Seinni strengurinn er read kóði en nota hann tímabundið

    //writeApi = channelsFylki[serial_number];

    //String channel = channelsFylki[serialNr];

    if (serial_number == 1.1){writeApi = "I9IJWNMPNF2TKY12";}
    else if (serial_number == 1.2){writeApi = "I9IJWNMPNF2TKY12";}
    else if (serial_number == 2.1){writeApi = "I9IJWNMPNF2TKY12";}
    else if (serial_number == 2.2){writeApi = "I9IJWNMPNF2TKY12";}
    else if (serial_number == 3.1){writeApi = "I9IJWNMPNF2TKY12";}
    else if (serial_number == 3.2){writeApi = "I9IJWNMPNF2TKY12";}
    else if (serial_number == 4.1){writeApi = "I9IJWNMPNF2TKY12";}
    else {printf("Serial number not found");}

    //Tímabundið
    writeApi = "I9IJWNMPNF2TKY12";
    //thingURL = thingURL + String(writeApi);

}

//Fall sem segir okkur hvort tölvan er tengd við netið
bool isConnected(){
    return WiFi.status() == WL_CONNECTED;
}


void Koltvioxid(float co2){
    if (isConnected()){
        ThingSpeak.writeField(2546046, 1, co2, writeApi);
    }
    else{
        Serial.print("Nær ekki tengingu til að senda koltvíoxíð");
    }

}

void Raki(float raki){
    if (isConnected()){
        ThingSpeak.writeField(2546046, 2, raki, writeApi);
    }
    else{
        Serial.print("Nær ekki tengingu til að senda koltvíoxíð");
    }
}

void Hitastig(float hiti){
    if (isConnected()){
        ThingSpeak.writeField(2546046, 3, hiti, writeApi);
    }
    else{
        Serial.print("Nær ekki tengingu til að senda koltvíoxíð");
    }
}
}
