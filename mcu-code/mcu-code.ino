#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>

// SparkFun sensor libraries
#include "SparkFun_ENS160.h"
#include "SparkFunBME280.h"
#include "Adafruit_SHT31.h"

// ------------------- Wi-Fi Credentials ------------------- //
const char *ssid     = "";
const char *password = "";

// IPAddress for the server once DNS is resolved
IPAddress ip;

// We'll include the device MAC for the HTTP request
String macAddress;

// ------------------- ENS160, BME280, SHT31 Instances ------------------- //
SparkFun_ENS160 myENS;
BME280 myBME280;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// ------------------- Power / Timing ------------------- //
// If you are NOT using GPIO3 as a sensor power-enable pin, set this to -1.
const int SENSOR_EN_PIN = 3;

// Watchdog every 2 min
static const uint32_t WDT_TIMEOUT_SEC = 120;

// Deep sleep every 1 min
static const uint64_t DEEP_SLEEP_US = 60ULL * 1000000ULL;

// ------------------------------------------------------------------
//  WATCHDOG
// ------------------------------------------------------------------
void initWatchdog() {
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_task_wdt_config_t wdt_config = {};
  wdt_config.timeout_ms = WDT_TIMEOUT_SEC * 1000;
  wdt_config.idle_core_mask = (1 << portNUM_PROCESSORS) - 1;
  wdt_config.trigger_panic = true;
  esp_task_wdt_init(&wdt_config);
#else
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
#endif
  esp_task_wdt_add(NULL); // current task
}

inline void feedWatchdog() {
  esp_task_wdt_reset();
}

// ------------------------------------------------------------------
//  HELPERS
// ------------------------------------------------------------------
bool connectWiFi(uint32_t timeoutMs = 30000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(500);
    feedWatchdog();
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
    return true;
  }

  Serial.println("WiFi connect timeout");
  return false;
}

String createSensorDataString(float sht31Temp,
                              float sht31Hum,
                              int   aqi,
                              int   tvoc,
                              int   eco2,
                              float bmeTemp,
                              float bmeHum,
                              float bmePress)
{
  String dataString = "";

  // SHT31
  dataString += "&soil_temp=" + (isnan(sht31Temp) ? "" : String(sht31Temp));
  dataString += "&soil_hum="  + (isnan(sht31Hum)  ? "" : String(sht31Hum));

  // ENS160
  dataString += "&aqi="  + (aqi  < 0 ? "" : String(aqi));
  dataString += "&tvoc=" + (tvoc < 0 ? "" : String(tvoc));
  dataString += "&eco2=" + (eco2 < 0 ? "" : String(eco2));

  // BME280
  dataString += "&temp="      + (isnan(bmeTemp)  ? "" : String(bmeTemp));
  dataString += "&hum="       + (isnan(bmeHum)   ? "" : String(bmeHum));
  dataString += "&pressure="  + (isnan(bmePress) ? "" : String(bmePress / 100.0)); // Pa -> hPa

  return dataString;
}

void goToDeepSleep() {
  Serial.println("Entering deep sleep for 60 seconds...");
  Serial.flush();

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_US);
  esp_deep_sleep_start();
}

// ------------------------------------------------------------------
//  SETUP (run once each wake)
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  initWatchdog();
  feedWatchdog();

  // Optional sensor power-enable pin
  if (SENSOR_EN_PIN >= 0) {
    pinMode(SENSOR_EN_PIN, OUTPUT);
    digitalWrite(SENSOR_EN_PIN, HIGH);
    delay(2000); // let sensors power up
    feedWatchdog();
  }

  // I2C
  Wire.begin(41, 40);

  // WiFi
  bool wifiOK = connectWiFi();

  // Resolve domain (only if WiFi connected)
  if (wifiOK) {
    if (WiFi.hostByName("opuntia.cc", ip)) {
      Serial.print("Resolved domain to IP: ");
      Serial.println(ip);
    } else {
      Serial.println("Failed to resolve domain name.");
      wifiOK = false;
    }
  }

  // Sensor init
  bool ensOK = myENS.begin();
  if (!ensOK) Serial.println("ENS160 did not begin.");
  else myENS.setOperatingMode(SFE_ENS160_STANDARD);

  bool bmeOK = myBME280.beginI2C();
  if (!bmeOK) Serial.println("BME280 did not respond.");

  bool shtOK = sht31.begin(0x44);
  if (!shtOK) Serial.println("SHT31 did not begin.");

  // MAC
  if (wifiOK) {
    macAddress = WiFi.macAddress();
    Serial.print("MAC Address: ");
    Serial.println(macAddress);
  } else {
    macAddress = "NA";
  }

  // Extra warm-up before reading (important after wake/power-up)
  delay(1500);
  feedWatchdog();

  // Read sensors (fallback-safe)
  float shtTemp = NAN, shtHum = NAN;
  int aqi = -1, tvoc = -1, eco2 = -1;
  float bmeTemp = NAN, bmeHum = NAN, bmePress = NAN;

  if (shtOK) {
    shtTemp = sht31.readTemperature();
    shtHum  = sht31.readHumidity();
    Serial.print("SHT31 Temp (C): "); Serial.println(shtTemp);
    Serial.print("SHT31 Hum (%): ");  Serial.println(shtHum);
  }

  if (ensOK) {
    aqi  = myENS.getAQI();
    tvoc = myENS.getTVOC();
    eco2 = myENS.getECO2();
    Serial.print("ENS160 -> AQI: "); Serial.print(aqi);
    Serial.print("  TVOC: ");        Serial.print(tvoc);
    Serial.print("  eCO2: ");        Serial.println(eco2);
  }

  if (bmeOK) {
    bmeTemp  = myBME280.readTempC();
    bmeHum   = myBME280.readFloatHumidity();
    bmePress = myBME280.readFloatPressure();
    Serial.print("BME280 Temp (C): ");    Serial.println(bmeTemp);
    Serial.print("BME280 Hum (%): ");     Serial.println(bmeHum);
    Serial.print("BME280 Pressure (Pa): "); Serial.println(bmePress);
  }

  feedWatchdog();

  // Send once per wake
  if (wifiOK) {
    String sensorData = createSensorDataString(
      shtTemp, shtHum, aqi, tvoc, eco2, bmeTemp, bmeHum, bmePress
    );

    Serial.println("Sensor data: " + sensorData);

    String serverPath = "http://" + ip.toString() + ":8089/sendData?mac="
                        + String(macAddress) + sensorData;

    Serial.println("Server path: " + serverPath);

    WiFiClient client;
    HTTPClient http;
    http.setConnectTimeout(10000);
    http.setTimeout(10000);
    http.begin(client, serverPath.c_str());

    feedWatchdog();
    int httpResponseCode = http.GET();
    feedWatchdog();

    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println("Server response: " + payload);
    } else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("Skipping send (WiFi/domain not ready).");
  }

  Serial.println("------------------------------------");

  // Sleep for 1 minute
  goToDeepSleep();
}

// ------------------------------------------------------------------
//  LOOP (not used; device sleeps from setup each cycle)
// ------------------------------------------------------------------
void loop() {
  // Not used
}
