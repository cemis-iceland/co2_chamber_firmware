#ifndef THINGSPEAK_
#define THINGSPEAK_

//WriteAPI = I9IJWNMPNF2TKY12
//ReadAPI = BI3IRYRDYRCIO4EO

namespace THINGSPEAK {
void SetupWiFi();
// Setup fyrir ThingSpeak aðgerðir.
void WriteAll(float co2, float raki, float hiti, float thryst);
void setup_ThingSpeak(int serial_number);
void Koltvioxid(float co2);
void Raki(float raki);
void Hitastig(float hiti);
void Thristingur(float thrist);
} // namespace THINGSPEAK
#endif