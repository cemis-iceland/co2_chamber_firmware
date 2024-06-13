#ifndef THINGSPEAK_
#define THINGSPEAK_

//WriteAPI = I9IJWNMPNF2TKY12
//ReadAPI = BI3IRYRDYRCIO4EO

namespace THINGSPEAK {
void SetupWiFi(float warmup_time);
void setup_ThingSpeak(int serial_number);
bool isConnected();
void WriteAll(float co2, float raki, float hiti, float thryst);
void Koltvioxid(float co2);
void Raki(float raki);
void Hitastig(float hiti);
void Thristingur(float thrist);
} //namespace THINGSPEAK
#endif