#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "SparkFun_ENS160.h"
#include "SparkFunBME280.h"
#include "Adafruit_SHT31.h"
#include "esp_adc_cal.h"

// Wi-Fi credentials - Changed to match old sensor code 
//const char *ssid = "MF928_D4117";
//const char *password = "FPZT6G2B3";

// const char *ssid = "LGMesaroof";    // Wi-Fi SSID
// const char *password = "12345678"; // Wi-Fi password
// const char *serverName = "http://54.89.237.246:8080/sendData"; // Server URL

String macAddress;

SparkFun_ENS160 myENS;
BME280 myBME280;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

unsigned long lastTime = 0;
unsigned long timerDelay = 2000;

#define BAT_ADC 2
float Voltage = 0.0;
uint32_t readADC_Cal(int ADC_Raw);

void setup()
{
    Wire.begin(18, 19); //SCL, SDA
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("Connected to WiFi");

    // check if the ENS sensor is connected
    if (!myENS.begin())
    {
        Serial.println("ENS160 did not begin.");
        while (1)
            ;
    }
    // check if the BME280 sensor is connected
    if (!myBME280.beginI2C())
    {
        Serial.println("BME280 did not respond.");
        while (1)
            ;
    }
    // check if the SHT31 sensor is connected
    if (!sht31.begin(0x44))
    {
        Serial.println("SHT31 did not begin.");
        while (1)
            ;
    }

    myENS.setOperatingMode(SFE_ENS160_STANDARD);
    sht31.begin(0x44);
    macAddress = WiFi.macAddress(); // Get MAC address

    // print macAddress
    Serial.println(macAddress);
}

String createSensorDataString(float dhtTemp, float dhtHum, float lux,
                              float sht31Temp, float sht31Hum,
                              int aqi, int tvoc, int eco2,
                              float bme280Hum, float bme280Press, float batteryVoltage)
{
    String dataString = "";

    // DHT Sensor (Old Device)
    dataString += "&dht_temp=" + (isnan(dhtTemp) ? "" : String(dhtTemp));
    dataString += "&dht_hum=" + (isnan(dhtHum) ? "" : String(dhtHum));

    // Light Sensor (Old Device)
    dataString += "&lux=" + (isnan(lux) ? "" : String(lux));

    // SHT31 Sensor (Common)
    dataString += "&sht31_temp=" + (isnan(sht31Temp) ? "" : String(sht31Temp));
    dataString += "&sht31_hum=" + (isnan(sht31Hum) ? "" : String(sht31Hum));

    // ENS160 Sensor (New Device)
    dataString += "&aqi=" + (aqi < 0 ? "" : String(aqi));
    dataString += "&tvoc=" + (tvoc < 0 ? "" : String(tvoc));
    dataString += "&eco2=" + (eco2 < 0 ? "" : String(eco2));

    // BME280 Sensor (New Device)
    dataString += "&bme280_hum=" + (isnan(bme280Hum) ? "" : String(bme280Hum));
    dataString += "&bme280_press=" + (isnan(bme280Press) ? "" : String(bme280Press / 100.0)); // Convert to hPa

    // Battery Voltage
    dataString += "&battery_voltage=" + String(batteryVoltage, 2);

    return dataString;
}

void loop()
{
    // Check if it's time to send data
    if ((millis() - lastTime) > timerDelay)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            WiFiClient client;
            HTTPClient http;

            // Read battery voltage
            Voltage = (readADC_Cal(analogRead(BAT_ADC))) * 2;
            float batteryVoltage = Voltage / 1000.0;

            //Read sensor data
            float sht31Temp = sht31.readTemperature();
            float sht31Hum = sht31.readHumidity();
            int aqi = myENS.getAQI();
            int tvoc = myENS.getTVOC();
            int eco2 = myENS.getECO2();
            float bme280Hum = myBME280.readFloatHumidity();
            float bme280Press = myBME280.readFloatPressure();


            String sensorData = createSensorDataString(NAN, NAN, NAN,                                    // Old device sensors
                                                       sht31.readTemperature(), sht31.readHumidity(),    // Common sensor
                                                       myENS.getAQI(), myENS.getTVOC(), myENS.getECO2(), // New device sensors
                                                       myBME280.readFloatHumidity(), myBME280.readFloatPressure(), batteryVoltage);

           //Print sensor data to serial monitor
           Serial.println("SHT31 Temperature: " + String(sht31Temp) + " °C" );
           Serial.println("SHT31 Humidity: " + String(sht31Hum) + " %");
           Serial.println("ENS160 Air Quality Index: " + String(aqi));
           Serial.println("ENS160 TVOC: " + String(tvoc) + " ppb");
           Serial.println("ENS160 eCO2: " + String(eco2) + " ppm");

            Serial.print("BME280 Humidity: ");
            Serial.print(bme280Hum);
            Serial.println(" %");

            Serial.print("BME280 Pressure: ");
            Serial.print(bme280Press / 100.0);
            Serial.println(" hPa");

            Serial.print("Battery Voltage: ");
            Serial.print(batteryVoltage);
            Serial.println(" V");

            //Send data to server
            String serverPath = String(serverName) + "?mac=" + String(macAddress) + sensorData;

            http.begin(client, serverPath.c_str());

            // Send HTTP GET request
            int httpResponseCode = http.GET();

            // Handle server response
            if (httpResponseCode > 0)
            {
                Serial.print("HTTP Response code: ");
                Serial.println(httpResponseCode);
                String payload = http.getString();
                Serial.println(payload);
            }
            else
            {
                Serial.print("Error code: ");
                Serial.println(httpResponseCode);
            }

            // Free resources
            http.end();
        }
        else
        {
            Serial.println("WiFi Disconnected");
        }

        lastTime = millis(); // Update last time data was sent
    }

    // Print battery voltage to serial monitor
    Voltage = (readADC_Cal(analogRead(BAT_ADC))) * 2;
    Serial.printf("%.2fV\n", Voltage / 1000.0); // Print Voltage (in V) with a new line
    delay(500);
}

uint32_t readADC_Cal(int ADC_Raw)
{
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}