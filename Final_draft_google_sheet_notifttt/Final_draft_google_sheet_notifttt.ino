#include <Arduino.h>
#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <ESP_Google_Sheet_Client.h>
#include <stdlib.h>
#include "time.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MHZ19_uart.h>
#include <HX711_ADC.h>
#include "DFRobot_ESP_PH.h"
#include "DHT.h"
#include <EEPROM.h>

// For SD/SD_MMC mounting helper
#include <GS_SDHelper.h>

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 0;

// WiFi credentials
#define WIFI_SSID "pc_home"
#define WIFI_PASSWORD "362136Pc"

// Google Sheets Credentials
#define PROJECT_ID "uropdatacollection"
#define CLIENT_EMAIL "urop-larevae-data@uropdatacollection.iam.gserviceaccount.com"
#define USER_EMAIL "menjiying@gmail.com"  // Change this to your email

char numberArray[20];
bool taskComplete = false;

// Service Account's private key
const char PRIVATE_KEY[] PROGMEM = "-----BEGIN PRIVATE KEY-----\nMIIEvgIB...END PRIVATE KEY-----\n";

// Sensor setup
MHZ19_uart mhz19;  // MHZ19 CO2 sensor
#define RX_PIN 13   // Serial tx pin no 
#define TX_PIN 14  // Serial rx pin no

HX711_ADC LoadCell(15, 16);  // Load cell pins
DFRobot_ESP_PH ph;
OneWire oneWire(5);  // OneWire for temperature sensor
DallasTemperature sensors(&oneWire);
DHT dht(29, DHT22);  // DHT22 sensor

float phValue, temperature = 25.0;
int SensValue = 0;

void tokenStatusCallback(TokenInfo info);
void readSensorsAndLog(FirebaseJson &valueRange, String &asString);

void setup() {
  Serial.begin(115200);
  delay(10);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");
  unsigned long ms = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
    if (millis() - ms > 10000)
      break;
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  GSheet.setTokenCallback(tokenStatusCallback);
  GSheet.setPrerefreshSeconds(10 * 60);
  GSheet.begin(CLIENT_EMAIL, PROJECT_ID, PRIVATE_KEY);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  Serial.print("Initial free heap: ");
  Serial.println(ESP.getFreeHeap());
  
  // Initialize sensors step by step
  // dht.begin();
  // Serial.print("Free heap after DHT22: ");
  // Serial.println(ESP.getFreeHeap());

  // sensors.begin();
  // Serial.print("Free heap after Dallas temp: ");
  // Serial.println(ESP.getFreeHeap());

  // mhz19.begin(RX_PIN, TX_PIN);
  // Serial.print("Free heap after MHZ19: ");
  // Serial.println(ESP.getFreeHeap());

  // EEPROM.begin(32);
  // Serial.print("Free heap after EEPROM: ");
  // Serial.println(ESP.getFreeHeap());

  // ph.begin();
  // Serial.print("Free heap after pH sensor: ");
  // Serial.println(ESP.getFreeHeap());

  // LoadCell.begin();
  // Serial.print("Free heap after Load Cell: ");
  // Serial.println(ESP.getFreeHeap());
}

void loop() {
  bool ready = GSheet.ready();
  if (ready && !taskComplete) {
    FirebaseJson response;

    Serial.println("\nCreate spreadsheet...");
    Serial.println("------------------------");

    FirebaseJson spreadsheet;
    spreadsheet.set("properties/title", "Sensor Data Log");
    spreadsheet.set("sheets/properties/gridProperties/rowCount", 100 );
    spreadsheet.set("sheets/properties/gridProperties/columnCount", 7);

    String spreadsheetId, spreadsheetURL;
    bool success = false;

    success = GSheet.create(&response /* returned response */, &spreadsheet /* spreadsheet object */, USER_EMAIL /* your email that this spreadsheet shared to */);
    response.toString(Serial, true);
    Serial.println();

    if (success) {
      FirebaseJsonData result;
      response.get(result, FPSTR("spreadsheetId"));
      if (result.success)
        spreadsheetId = result.to<const char *>();

      result.clear();
      response.get(result, FPSTR("spreadsheetUrl"));
      if (result.success) {
        spreadsheetURL = result.to<const char *>();
        Serial.println("\nThe spreadsheet URL");
        Serial.println(spreadsheetURL);
      }

      struct tm timeinfo;
      char timeStringBuff[50];
      String asString;
      char buffer[40];

      FirebaseJson valueRange;
      for (int counter = 0; counter < 10; counter++) {
        Serial.println("\nUpdate spreadsheet values...");
        Serial.println("------------------------------");
        if (!getLocalTime(&timeinfo)) {
          Serial.println("Failed to obtain time");
          return;
        }
        strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
        asString = timeStringBuff;
        asString.replace(" ", "-");

        readSensorsAndLog(valueRange, asString);

        success = GSheet.values.update(&response, spreadsheetId, "Sheet1!A1:G1000", &valueRange);
        response.toString(Serial, true);
        Serial.println();
        delay(5000);
      }

      if (success) {
        Serial.println("\nGet spreadsheet values...");
        Serial.println("------------------------------");

        success = GSheet.values.get(&response, spreadsheetId, "Sheet1!A1:B10");
        response.toString(Serial, true);
        Serial.println();

        Serial.println(ESP.getFreeHeap());
      }
    }
    taskComplete = true;
  }
}

void readSensorsAndLog(FirebaseJson &valueRange, String &asString) {
  // Load cell data
  float weight = LoadCell.getData();
  // pH sensor data
  phValue = ph.readPH(analogRead(9) / 4096.0 * 3300, temperature);
  // Temperature sensor data
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  // CO2 sensor data
  int co2ppm = mhz19.getCO2PPM();
  // DHT22 data
  float dhtTemp = dht.readTemperature();
  float dhtHum = dht.readHumidity();

  // Fill value range with sensor data
  valueRange.set("values/[0]/[0]", asString);
  valueRange.set("values/[0]/[1]", String(weight));
  valueRange.set("values/[0]/[2]", String(phValue));
  valueRange.set("values/[0]/[3]", String(tempC));
  valueRange.set("values/[0]/[4]", String(co2ppm));
  valueRange.set("values/[0]/[5]", String(dhtHum));
  valueRange.set("values/[0]/[6]", String(dhtTemp));
}

void tokenStatusCallback(TokenInfo info) {
  if (info.status == esp_signer_token_status_error) {
    Serial.printf("Token error: %s\n", GSheet.getTokenError(info).c_str());
  } else {
    Serial.printf("Token status: %s\n", GSheet.getTokenStatus(info).c_str());
  }
}
