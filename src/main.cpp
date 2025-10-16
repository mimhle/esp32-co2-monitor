#include "esp32-hal-tinyusb.h"
#include "secrets.h"
#include "utils.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <SensirionI2cScd4x.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

#define DIVIDER_RATIO 2.0129

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define LED_PIN 48
#define BAT_PIN 7

Preferences preferences;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
SensirionI2cScd4x sensor;

WiFiClient espClient;
PubSubClient client(espClient);
GFXcanvas1 canvas(SCREEN_WIDTH, SCREEN_HEIGHT);

void sleep() {
    esp_light_sleep_start();
    delay(10);
    pinMode(BAT_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(BAT_PIN, ADC_11db);
}

struct ConnectArgs {
    String ssid;
    String pass;
};
volatile bool wifiConnectInProgress = false;
volatile wl_status_t wifiLastStatus = WL_IDLE_STATUS;
volatile uint8_t wifiLastReason = 0;
volatile bool wifiConnected = false;
uint8_t wifiIndex = 0;
String wifiSSID;

void setupWifi() {
    wifi_country_t country = {"VN", 1, 13, WIFI_COUNTRY_POLICY_MANUAL};
    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(84);
    esp_wifi_set_country(&country);
    WiFi.setSleep(false);
    WiFi.persistent(false);
    WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t info) {
        if (e == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
            wifiLastReason = info.wifi_sta_disconnected.reason;
            wifiLastStatus = WL_CONNECT_FAILED;
            wifiConnectInProgress = false;
            wifiConnected = false;
        } else if (e == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
            wifiLastStatus = WL_CONNECTED;
            wifiConnectInProgress = false;
            wifiConnected = true;
        }
    });
}

void startWifiConnectAsync(const String &ssid, const String &pass) {
    if (wifiConnectInProgress)
        return;
    wifiConnectInProgress = true;
    wifiLastStatus = WL_IDLE_STATUS;
    wifiLastReason = 0;
    wifiConnected = false;

    WiFi.disconnect(true, true);
    WiFi.begin(ssid.c_str(), pass.c_str());

    auto *args = new ConnectArgs{ssid, pass};
    xTaskCreate(
        [](void *p) {
            ConnectArgs *a = static_cast<ConnectArgs *>(p);
            uint32_t start = millis();
            while (wifiConnectInProgress && millis() - start < 10000) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            if (wifiLastStatus == WL_CONNECTED) {
                // Successfully connected
                wifiConnected = true;
            }
            delete a;
            vTaskDelete(NULL);
        },
        "wifiConnect", 4096, args, 1, nullptr);
}

void connectMQTT() {
    if (client.connected())
        return;
    String cid = "ESP32Client-";
    cid += String(random(0xffff), HEX);
    client.setServer(MQTT_BROKER, MQTT_PORT);
    if (client.connect(cid.c_str())) {
        // Connected
    } else {
        delay(2000);
    }
}

const float EMA_ALPHA = 0.2f;
float ema_vbat = NAN;
float last_min_mV = 1e9;
const float HYST_mV = 6.0f;
const int REG_POINTS = 20;
uint32_t lastSample = 0;

struct Samp {
    uint32_t t_ms;
    float soc;
};
Samp buf[REG_POINTS];
int bufCount = 0, bufHead = 0;

void pushPoint(uint32_t t_ms, float soc) {
    buf[bufHead] = {t_ms, soc};
    bufHead = (bufHead + 1) % REG_POINTS;
    if (bufCount < REG_POINTS)
        bufCount++;
}

bool slopeSocPerSec(float &slope_out) {
    if (bufCount < 2)
        return false; // need enough points
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    // normalize time to the first point to improve numerics
    uint32_t t0 = buf[(bufHead - bufCount + REG_POINTS) % REG_POINTS].t_ms;
    for (int i = 0; i < bufCount; i++) {
        int idx = (bufHead - bufCount + i + REG_POINTS) % REG_POINTS;
        double x = (buf[idx].t_ms - t0) / 1000.0; // seconds
        double y = buf[idx].soc;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    double n = bufCount;
    double denom = n * sxx - sx * sx;
    if (denom <= 1e-6)
        return false;
    double m = (n * sxy - sx * sy) / denom; // % per second
    slope_out = (float)m;
    return true;
}

float readBatteryVoltage() {
    float vbat = (analogReadMilliVolts(BAT_PIN) / 1000.0f) * DIVIDER_RATIO;
    if (isnan(ema_vbat))
        ema_vbat = vbat;
    else
        ema_vbat = EMA_ALPHA * vbat + (1.0f - EMA_ALPHA) * ema_vbat;

    float mv_pin_ema = (ema_vbat * 1000.0f) / DIVIDER_RATIO;
    bool accepted = false;
    if (mv_pin_ema + HYST_mV <= last_min_mV) {
        last_min_mV = mv_pin_ema;
        float soc = mapFloat(min(max(ema_vbat, 3.3f), 4.2f), 3.3f, 4.2f, 0.0f, 100.0f);
        pushPoint(millis(), soc);
        accepted = true;
    }
    float slope_pct_per_s;
    bool haveSlope = slopeSocPerSec(slope_pct_per_s) && slope_pct_per_s < -1e-4f; // must be decreasing
    float soc_now = mapFloat(min(max(ema_vbat, 3.3f), 4.2f), 3.3f, 4.2f, 0.0f, 100.0f);
    float hours_left = NAN;
    if (haveSlope) {
        float remaining_pct = max(0.0f, soc_now);
        float sec_left = remaining_pct / (-slope_pct_per_s); // seconds
        hours_left = sec_left / 3600.0f;
    }

    canvas.setTextSize(1);
    canvas.setCursor(70, 8);
    if (haveSlope) {
        unsigned long total_s = (unsigned long)(hours_left * 3600.0f + 0.5f);
        unsigned h = total_s / 3600;
        unsigned m = (total_s % 3600) / 60;
        unsigned s = total_s % 60;
        canvas.printf("%02u:%02u:%02u", h, m, s);
    } else {
        canvas.print("----");
    }

    return ema_vbat;
}

int16_t sensor_error = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Serial started");

    setupWifi();

    preferences.begin("co2", false);

    esp_sleep_enable_timer_wakeup(5ULL * 1000000ULL);

    Wire.begin(8, 9);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    sensor.begin(Wire, SCD41_I2C_ADDR_62);

    delay(50);
    sensor_error = sensor.wakeUp();
    sensor_error = sensor.startPeriodicMeasurement();

    pinMode(LED_PIN, OUTPUT);

    pinMode(BAT_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(BAT_PIN, ADC_11db);

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setRotation(2);
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.println("Loading...");
    if (sensor_error != 0) {
        display.print("Sensor error: ");
        display.print(sensor_error);
    }
    display.display();
}

void loop() {
    if (wifiConnected) {
        if (!client.connected()) {
            connectMQTT();
        }
        client.loop();
    }

    bool dataReady = false;
    uint16_t co2Concentration = 0;
    float temperature = 0.0;
    float relativeHumidity = 0.0;
    uint32_t pressure = 0;

    if (!wifiConnected) {
        canvas.fillRect(0, 0, 69, 10, 0);
        drawWifiIcon(canvas, 0, 0, true);
        canvas.setTextSize(1);
        canvas.setCursor(11, 0);
        canvas.print(trucateText(access_points[wifiIndex], 9));
        display.drawBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height(), 1, 0);
        display.display();
        startWifiConnectAsync(String(access_points[wifiIndex]), String(passwords[wifiIndex]));
        while (wifiConnectInProgress) {
            delay(500);
            Serial.print(".");
        }
        Serial.printf("SSID: %s, Status: %d, Reason: %d\n", access_points[wifiIndex], wifiLastStatus, wifiLastReason);
        if (wifiLastStatus == WL_CONNECTED) {
            wifiConnected = true;
            wifiSSID = access_points[wifiIndex];
            connectMQTT();
        } else {
            wifiConnected = false;
            wifiSSID = "";
            wifiIndex = (wifiIndex + 1) % (sizeof(access_points) / sizeof(access_points[0]));
        }
        delay(1000);
    }

    sensor.getDataReadyStatus(dataReady);
    if (!dataReady) {
        // if (digitalRead(btn.pin()) != LOW)
        //     sleep();
        delay(100);
        return;
    }

    digitalWrite(LED_PIN, LOW);

    display.clearDisplay();
    canvas.fillScreen(0);

    sensor.readMeasurement(co2Concentration, temperature, relativeHumidity);
    float vbat = readBatteryVoltage();

    canvas.setTextSize(2);
    canvas.setCursor(0, 15);
    canvas.print(co2Concentration);
    canvas.setTextSize(1);
    canvas.print("ppm");

    drawBatteryGauge(canvas, 70, 0, mapFloat(min(max(vbat, 3.3f), 4.2f), 3.3f, 4.2f, 0.0f, 100.0f), vbat < 3.3f);
    if (wifiConnected) {
        drawWifiIcon(canvas, 0, 0, false);
        canvas.setTextSize(1);
        canvas.setCursor(11, 0);
        canvas.print(trucateText(wifiSSID, 9));
    }

    canvas.setTextSize(1);
    canvas.setCursor(84, 0);
    canvas.print(vbat);
    canvas.print("v");

    canvas.setCursor(70, 16);
    canvas.print(temperature);
    canvas.print("C");

    canvas.setCursor(70, 24);
    canvas.print(relativeHumidity);
    canvas.print("RH");

    display.drawBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height(), 1, 0);
    display.display();

    if (wifiConnected && client.connected()) {
        JsonDocument doc;
        doc["co2"] = co2Concentration;
        doc["temp"] = temperature;
        doc["rh"] = relativeHumidity;
        doc["vbat"] = vbat;
        char buf[256];
        size_t n = serializeJson(doc, buf);
        client.publish("home/co2", buf, n);
    }

    digitalWrite(LED_PIN, HIGH);
    delay(10);
    digitalWrite(LED_PIN, LOW);
    if (!wifiConnected || !client.connected()) {
        delay(100);
        digitalWrite(LED_PIN, HIGH);
        delay(10);
        digitalWrite(LED_PIN, LOW);
    }
}
