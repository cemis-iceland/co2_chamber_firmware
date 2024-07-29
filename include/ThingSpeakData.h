#ifndef THINGSPEAK_
#define THINGSPEAK_


namespace THINGSPEAK {
void SetupWiFi(float warmup_time);
void setup_ThingSpeak();
bool isConnected();
void WriteAll(float co2, float raki, float hiti, float thryst, bool VALVES_CLOSED);
void Koltvioxid(int channelNumber,float co2);
void Raki(int channelNumber,float raki);
void Hitastig(int channelNumber,float hiti);
void Thristingur(int channelNumber,float thrist);
void Status(String STATUS);
} // namespace THINGSPEAK
#endif