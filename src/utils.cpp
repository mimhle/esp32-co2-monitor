#include "utils.h"

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max == in_min)
        return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drawBatteryGauge(GFXcanvas1 &canvas, int16_t x, int16_t y, uint8_t percent, bool lowBatteryAlert) {
    const uint8_t BATT_W = 10;
    const uint8_t BATT_H = 7;
    const uint8_t NUB_W = 2;
    const uint8_t NUB_H = 3;

    canvas.drawRect(x, y, BATT_W, BATT_H, 0xFFFF);

    int16_t nubX = x + BATT_W;
    int16_t nubY = y + (BATT_H - NUB_H) / 2;
    canvas.fillRect(nubX, nubY, NUB_W, NUB_H, 0xFFFF);

    int16_t innerX = x + 1;
    int16_t innerY = y + 1;
    int16_t innerW = BATT_W - 2;
    int16_t innerH = BATT_H - 2;

    int16_t fillW = (int16_t)((innerW * (uint16_t)percent) / 100);
    if (fillW > 0) {
        canvas.fillRect(innerX, innerY, fillW, innerH, 0xFFFF);
    }

    if (lowBatteryAlert) {
        canvas.drawLine(innerX, innerY, innerX + innerW, innerY + innerH, 0xFFFF);
    }
}

void drawWifiIcon(GFXcanvas1 &canvas, int16_t x, int16_t y, bool connecting) {
    const int16_t ICON_H = 7;
    const int16_t MAX_BARS = 4;

    for (int i = 0; i < MAX_BARS; i += 1 + connecting) {
        canvas.drawFastVLine(x + 1 + i * 2, y + ICON_H - 1 - (i + 1), i + 1, 0xFFFF);
    }
}

String trucateText(String &str, size_t maxLen) {
    if (str.length() > maxLen) {
        str = str.substring(0, maxLen - 3) + "...";
    }
    return str;
}

char* trucateText(const char* str, size_t maxLen) {
    static char buf[32];
    strncpy(buf, str, sizeof(buf));
    buf[sizeof(buf) - 1] = '\0';
    if (strlen(buf) > maxLen) {
        buf[maxLen - 3] = '\0';
        strcat(buf, "...");
    }
    return buf;
}