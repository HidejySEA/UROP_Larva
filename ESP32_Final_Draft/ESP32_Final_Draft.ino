#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MHZ19_uart.h>
#include <HX711_ADC.h>
#include "DFRobot_ESP_PH.h"
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#include "DHT.h"
#endif

// WiFi credentials
const char* ssid     = "notyouriphone";
const char* password = "hidejy123";

// IFTTT server information
const char* resource = "https://maker.ifttt.com/trigger/ESP32_test/with/key/dhQ0R8oARgUE5M9wfjWahi";
const char* server = "maker.ifttt.com";

// MHZ19 CO2 sensor pins
const int rx_pin = 1; // Serial tx pin no 
const int tx_pin = 2; // Serial rx pin no  (these two ports need to be reversed )
MHZ19_uart mhz19;

// HX711 load cell pins and initialization
const int HX711_dout = 3; // MCU > HX711 dout pin
const int HX711_sck = 4;  // MCU > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

// Temperature sensor initialization
#define ONE_WIRE_BUS 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// pH sensor setup
DFRobot_ESP_PH ph;
#define ESPADC 4096.0   // the esp Analog Digital Conversion value
#define ESPVOLTAGE 3300 // the esp voltage supply value
#define PH_PIN 9      // the esp gpio data pin number (i forgot to connect to which one)
float voltage, phValue, temperature = 25;

// Capacitive Soil Moisture Sensor
#define m_sensorPin1 6
#define m_sensorPin2 7
#define m_sensorPin3 8

byte analog_pinsOut_moisture[] = {m_sensorPin1, m_sensorPin2, m_sensorPin3};
const int num_of_msensors = sizeof(analog_pinsOut_moisture) / sizeof(analog_pinsOut_moisture[0]);
float measured_moisture_Array[num_of_msensors]; // Store an instance of all moisture measured

// DHT 22
#define DHTPIN 29     // Digital pin connected to the DHT sensor #2 located at the Outlet
#define DHTTYPE DHT22   // DHT 22
DHT dht(DHTPIN, DHTTYPE); // Initilialize DHT sensor
float dhtTemperature = 0;
float dhtHumidity = 0;

// Timing settings
unsigned long previousMillis = 0; 
const long interval = 60000; // Interval to wait for (milliseconds) - 1 minute

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
  // pH sensor setup
  EEPROM.begin(32); // Needed to permit storage of calibration value in EEPROM
  ph.begin();
  }
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

    // Read temperature
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
    readMoistureSensors();
    for (int i = 0; i < num_of_msensors; i++) {
      Serial.print("Moisture Sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(measured_moisture_Array[i]);
    }
    // Read DHT sensor
    readDHTSensor();
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
    makeIFTTTRequest();
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

void readMoistureSensors() {
  for (int i = 0; i < num_of_msensors; i++) {
    measured_moisture_Array[i] = analogRead(analog_pinsOut_moisture[i]);
    // Serial.print("Moisture Sensor ");
    // Serial.print(i + 1);
    // Serial.print(": ");
    // Serial.println(measured_moisture_Array[i]);
  }
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

void makeIFTTTRequest() {
  Serial.print("Connecting to "); 
  Serial.print(server);

  WiFiClient client;
  int retries = 5;
  while(!client.connect(server, 80) && (retries-- > 0)) {
    Serial.print(".");
  }
  Serial.println();
  if(!client.connected()) {
    Serial.println("Failed to connect...");
    return;
  }

  Serial.print("Request resource: ");
  Serial.println(resource);

  // Get temperature
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // Get CO2 and temperature from MH-Z16 sensor
  int co2ppm = mhz19.getCO2PPM();
  int tempco2 = mhz19.getTemperature();

  // Get weight from HX711 load cell
  float weight = LoadCell.getData();

  // Get pH value from pH sensor
  float pHValue = ph.readPH(voltage, temperature);

  // Get moisture values from moisture sensors
  float moisture1 = measured_moisture_Array[0];
  float moisture2 = measured_moisture_Array[1];
  float moisture3 = measured_moisture_Array[2];

  // Read DHT sensor values (already done in loop)
  float dhtTemp = dhtTemperature;
  float dhtHum = dhtHumidity;

// 1st request: CO2, CO2 temp, DHT temperature
  String jsonObject1 = String("{\"value1\":\"") + co2ppm + "\",\"value2\":\"" + tempco2
                        + "\",\"value3\":\"" + dhtTemp + "\"}";
  sendIFTTTRequest(client, jsonObject1);

  // 2nd request: Weight, pH, DHT humidity
  String jsonObject2 = String("{\"value1\":\"") + weight + "\",\"value2\":\"" + pHValue
                        + "\",\"value3\":\"" + dhtHum + "\"}";
  sendIFTTTRequest(client, jsonObject2);

  // 3rd request: Moisture values (moisture1, moisture2, moisture3)
  String jsonObject3 = String("{\"value1\":\"") + moisture1 + "\",\"value2\":\"" + moisture2
                        + "\",\"value3\":\"" + moisture3 + "\"}";
  sendIFTTTRequest(client, jsonObject3);

  Serial.println("\nclosing connection");
  client.stop();
}

// Function to send individual requests
void sendIFTTTRequest(WiFiClient &client, String jsonObject) {
  // Ensure the client is connected
  if (!client.connected()) {
    Serial.println("Reconnecting...");
    if (!client.connect(server, 80)) {
      Serial.println("Failed to reconnect...");
      return;
    }
  }

  // Prepare the HTTP POST request
  client.println(String("POST ") + resource + " HTTP/1.1");
  client.println(String("Host: ") + server); 
  client.println("Connection: close\r\nContent-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonObject.length());
  client.println();
  client.println(jsonObject);

  // Wait for response
  int timeout = 5 * 10; // 5 seconds             
  while(!client.available() && (timeout-- > 0)){
    delay(100);
  }
  if(!client.available()) {
    Serial.println("No response...");
    return;
  }

  // Print the server's response for debugging
  while(client.available()){
    Serial.write(client.read());
  }
}
