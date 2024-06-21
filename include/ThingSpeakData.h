#ifndef THINGSPEAK_
#define THINGSPEAK_


namespace THINGSPEAK {
void SetupWiFi(float warmup_time);
void setup_ThingSpeak(int serial_number);
bool isConnected();
void WriteAll(float co2, float raki, float hiti, float thryst, bool VALVES_CLOSED);
void Koltvioxid(float co2);
void Raki(float raki);
void Hitastig(float hiti);
void Thristingur(float thrist);
} //namespace THINGSPEAK
#endif