#ifndef THINGSPEAK_
#define THINGSPEAK_

//WriteAPI = I9IJWNMPNF2TKY12
//ReadAPI = BI3IRYRDYRCIO4EO

namespace THINGSPEAK {
void SetupWiFi(const char* ssid, const char* password);

// Setup fyrir ThingSpeak aðgerðir.
void setup_ThingSpeak(int serial_number);

void Koltvioxid(float co2);

void Raki(float raki);

void Hitastig(float hiti);
}
#endif