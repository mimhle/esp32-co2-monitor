#include <Adafruit_GFX.h>
#include <WiFi.h>

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void drawBatteryGauge(GFXcanvas1 &canvas, int16_t x, int16_t y, uint8_t percent, bool lowBatteryAlert);
void drawWifiIcon(GFXcanvas1 &canvas, int16_t x, int16_t y, bool connecting);
String trucateText(String &str, size_t maxLen);
char* trucateText(const char* str, size_t maxLen);