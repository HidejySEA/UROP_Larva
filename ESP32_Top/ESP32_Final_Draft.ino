#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MHZ19_uart.h>
#include <HX711_ADC.h>
#include "DFRobot_ESP_PH.h"
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#include "DHT.h"
#include <HTTPClient.h>
#endif

// WiFi credentials
const char* ssid     = "pc_home";
const char* password = "362136Pc";

// Google script Web_App_URL.
String Web_App_URL = "https://script.google.com/macros/s/AKfycbwQqygU4waYcLD35KX0dw68z_hXzK_1FgT4MM2peYt-iqu1VKyAb5jBwnlAkuke2y1Y/exec";
// MHZ19 CO2 sensor pins
const int rx_pin = 15; // Serial tx pin no 
const int tx_pin = 16; // Serial rx pin no  (these two ports need to be reversed )
MHZ19_uart mhz19;

// HX711 load cell pins and initialization
const int HX711_dout = 4; // MCU > HX711 dout pin
const int HX711_sck =  5;  // MCU > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

// Temperature sensor initialization
#define ONE_WIRE_BUS 6
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// pH sensor setup
DFRobot_ESP_PH ph;
#define ESPADC 4096.0   // the esp Analog Digital Conversion value
#define ESPVOLTAGE 3300 // the esp voltage supply value
#define PH_PIN 10      // the esp gpio data pin number (i forgot to connect to which one)
float voltage, phValue, temperature = 25;

// Capacitive Soil Moisture Sensor
#define m_sensorPin1 11
float measured_moisture; // Store the moisture measured


// DHT 22
#define DHTPIN 35    // Digital pin connected to the DHT sensor #2 located at the Outlet
#define DHTTYPE DHT22   // DHT 22
DHT dht(DHTPIN, DHTTYPE); // Initilialize DHT sensor
float dhtTemperature = 0;
float dhtHumidity = 0;

//MQ4 MQ137 ammonia and methane sensor
#define MQ4_PIN 37    // Analog pin connected to MQ-4 methane sensor
#define MQ137_PIN 36  // Analog pin connected to MQ-137 ammonia sensor

// Fan control pins
#define FAN1_PWM_PIN 17  // PWM pin for Fan 1
#define FAN2_PWM_PIN 18  // PWM pin for Fan 2


// Temperature threshold for fan control
#define TEMP_LOW 25    // Low temperature threshold
#define TEMP_MEDIUM 30 // Medium temperature threshold
#define TEMP_HIGH 35   // High temperature threshold

// Timing settings
unsigned long previousMillis = 0; 
const long interval = 60000; // Interval to wait for (milliseconds) - 1 minute
// Fan speed
int fan1Speed = 0;  // Variable to store Fan 1 speed (PWM value)
int fan2Speed = 0;  // Variable to store Fan 2 speed (PWM value)
// mq series
float mq4Voltage;   // Global variable for methane sensor voltage
float mq137Voltage; // Global variable for ammonia sensor voltage

//--------------------------------------SETUP---------------------------------------------//
void setup() {
  Serial.begin(115200); 
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  // WiFi setup
  initWifi();
  
  // Initialize DHT sensor
  dht.begin();
  
  // Temperature sensor setup
  pinMode(2, INPUT_PULLUP);
  sensors.begin();

  // MHZ19 CO2 sensor setup
  mhz19.begin(rx_pin, tx_pin);
  Serial.println("MH-Z19 is warming up now.");
  delay(10 * 1000); // Warming up MH-Z19

  // HX711 setup
  float calibrationValue = 696.0; // Calibration value
  LoadCell.begin();
  unsigned long stabilizingtime = 2000; // Tare precision can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; // Set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  } else {
    LoadCell.setCalFactor(calibrationValue); // Set calibration factor (float)
    Serial.println("Startup is complete");
  }
  // pH sensor setup
  EEPROM.begin(32); // Needed to permit storage of calibration value in EEPROM
  ph.begin();

// Set initial fan speed to 0 (fans off)
  analogWrite(FAN1_PWM_PIN, 0);
  analogWrite(FAN2_PWM_PIN, 0);
}
void loop() {
  unsigned long currentMillis = millis();
  static boolean newDataReady = 0;
  const int serialPrintInterval = 10000; // Set to 10 seconds (10000 milliseconds)
  
  // Check for new data from HX711
  if (LoadCell.update()) newDataReady = true;

  // Get smoothed value from the dataset and print it every 10 seconds
  if (newDataReady && millis() - previousMillis >= serialPrintInterval) {
    previousMillis = millis(); // Reset the last print time
    float weight = LoadCell.getData();
    Serial.print("Load_cell output value: ");
    Serial.println(weight);
    newDataReady = 0;

    // pH sensor reading
    voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // Read the voltage
    phValue = ph.readPH(voltage, temperature); // Convert voltage to pH with temperature compensation
    Serial.print("pH:");
    Serial.println(phValue, 4);
    
    // Read temperature probe
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    Serial.print("Temperature (C): ");
    Serial.println(tempC);

    // Read CO2 sensor
    int co2ppm = mhz19.getCO2PPM();
    int tempco2 = mhz19.getTemperature();
    Serial.print("CO2 PPM: ");
    Serial.println(co2ppm);
    Serial.print("MHZ19 Temp (C): ");
    Serial.println(tempco2);

    // Read moisture sensors
    readMoistureSensor();
    Serial.print("Moisture Sensor: ");
    Serial.println(measured_moisture);


    // Read DHT sensor
    readDHTSensor();

    // Read MQ4 (Methane) sensor
    mq4Voltage = (analogRead(MQ4_PIN) / 4095.0) * 3.3;  // Calculate methane sensor voltage
    Serial.print("MQ-4 Methane Sensor Voltage: ");
    Serial.println(mq4Voltage);

    // Read MQ137 (Ammonia) sensor
    mq137Voltage = (analogRead(MQ137_PIN) / 4095.0) * 3.3;  // Calculate ammonia sensor voltage
    Serial.print("MQ-137 Ammonia Sensor Voltage: ");
    Serial.println(mq137Voltage);

    // Fan speed control based on temperature
    controlFans(tempC);
  }

  // Receive command from serial terminal, send 't' to initiate tare operation
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // Check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

  // Check if it's time to publish a new message
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the last time a message was sent
    // Upload data to Google Sheets
    uploadDataToGoogleSheets(); // Call to upload data to Google Sheets
  }
}


void initWifi() {
  Serial.print("Connecting to: "); 
  Serial.print(ssid);
  WiFi.begin(ssid, password); 

  int timeout = 10 * 4; // 10 seconds
  while(WiFi.status() != WL_CONNECTED  && (timeout-- > 0)) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("");

  if(WiFi.status() != WL_CONNECTED) {
     Serial.println("Failed to connect");
     return;
  }

  Serial.print("WiFi connected in: "); 
  Serial.print(millis());
  Serial.print(", IP address: "); 
  Serial.println(WiFi.localIP());
}


void readMoistureSensor() {
  measured_moisture = analogRead(m_sensorPin1);
  Serial.print("Moisture Sensor: ");
  Serial.println(measured_moisture);
}

void readDHTSensor() {
    dhtHumidity = dht.readHumidity();
    dhtTemperature = dht.readTemperature();

    // Check if any reads failed and exit early (to try again next time).
    if (isnan(dhtHumidity) || isnan(dhtTemperature)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }
}

void controlFans(float temperature) {
  if (temperature > TEMP_HIGH) {
    // Full speed (100%)
    fan1Speed = 255;
    fan2Speed = 255;
    analogWrite(FAN1_PWM_PIN, fan1Speed);
    analogWrite(FAN2_PWM_PIN, fan2Speed);
    Serial.println("Fans set to 100% speed (Full Power)");
  } else if (temperature > TEMP_MEDIUM && temperature <= TEMP_HIGH) {
    // 75% speed
    fan1Speed = 192;
    fan2Speed = 192;
    analogWrite(FAN1_PWM_PIN, fan1Speed);
    analogWrite(FAN2_PWM_PIN, fan2Speed);
    Serial.println("Fans set to 75% speed");
  } else if (temperature > TEMP_LOW && temperature <= TEMP_MEDIUM) {
    // 50% speed
    fan1Speed = 128;
    fan2Speed = 128;
    analogWrite(FAN1_PWM_PIN, fan1Speed);
    analogWrite(FAN2_PWM_PIN, fan2Speed);
    Serial.println("Fans set to 50% speed");
  } else {
    // 25% speed
    fan1Speed = 64;
    fan2Speed = 64;
    analogWrite(FAN1_PWM_PIN, fan1Speed);
    analogWrite(FAN2_PWM_PIN, fan2Speed);
    Serial.println("Fans set to 25% speed");
  }
}


// Replace the current makeIFTTTRequest() function with this updated version

void uploadDataToGoogleSheets() {
  Serial.println("Uploading data to Google Sheets...");

  if (WiFi.status() == WL_CONNECTED) {
    // Create a URL for sending data to Google Sheets
    String sendDataURL = Web_App_URL + "?sts=write";
    sendDataURL += "&temp=" + String(sensors.getTempCByIndex(0)); // Temperature probe
    sendDataURL += "&co2=" + String(mhz19.getCO2PPM()); // CO2 PPM
    sendDataURL += "&pH=" + String(phValue, 4); // pH value
    sendDataURL += "&moisture=" + String(measured_moisture); // Soil moisture
    sendDataURL += "&weight=" + String(LoadCell.getData()); // Load cell weight
    sendDataURL += "&humidity=" + String(dhtHumidity); // dht22 humidity
    sendDataURL += "&methane=" + String(mq4Voltage); // methane
    sendDataURL += "&ammonia=" + String(mq137Voltage); // ammonia
    sendDataURL += "&fan1=" + String(fan1Speed);  // Fan 1 speed
    sendDataURL += "&fan2=" + String(fan2Speed);  // Fan 2 speed


    // Log the URL for debugging
    Serial.println("URL: " + sendDataURL);

    // Send the HTTP GET request to Google Sheets
    HTTPClient http;
    http.begin(sendDataURL.c_str());
    int httpCode = http.GET();
    
    // Check the HTTP status code
    if (httpCode > 0) {
      Serial.print("HTTP Status Code: ");
      Serial.println(httpCode);
      String payload = http.getString();
      Serial.println("Payload: " + payload);
    } else {
      Serial.println("Error in HTTP request");
    }
    
    http.end(); // End the HTTP request
  } else {
    Serial.println("WiFi not connected");
  }
}