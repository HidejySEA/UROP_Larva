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
#include <UniversalTelegramBot.h>
#include <WiFiClientSecure.h>
#endif

// WiFi credentials
const char* ssid     = "notyouriphone";
const char* password = "hidejy123";

// Google script Web_App_URL.
String Web_App_URL = "https://script.google.com/macros/s/AKfycbyI-P0pGyJTJ4JevCI6MkAdXxH5J5JCeb78t4gC7NDbqzmqd_S7xIWKz0yifErPjG6log/exec";
// MHZ19 CO2 sensor pins
const int rx_pin = 16; // Serial tx pin no 
const int tx_pin = 17; // Serial rx pin no  (these two ports need to be reversed )
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
bool inPHCalibrationMode = false;  // Track calibration mode

// Capacitive Soil Moisture Sensor
#define m_sensorPin1 11
float measured_moisture; // Store the moisture measured
float output_moisture; // store the output moisture


// DHT 22
#define DHTPIN 36    // Digital pin connected to the DHT sensor #2 located at the Outlet
#define DHTTYPE DHT22   // DHT 22
DHT dht(DHTPIN, DHTTYPE); // Initilialize DHT sensor
float dhtTemperature = 0;
float dhtHumidity = 0;

// Fan control pins
#define FAN1_PWM_PIN 41  // PWM pin for Fan 1
#define FAN2_PWM_PIN 42  // PWM pin for Fan 2
const int fan1pin = 41;  // 16 corresponds to GPIO 16
const int fan2pin = 42;

// Temperature threshold for fan control
#define TEMP_LOW 20    // Low temperature threshold
#define TEMP_MEDIUM 25 // Medium temperature threshold
#define TEMP_HIGH 30   // High temperature threshold

// Timing settings
unsigned long previousMillisSerial = 0;  // Timing for serial print
unsigned long previousMillisUpload = 0;  // Timing for uploading to Google Sheets
const long serialInterval = 10000;  // Interval for serial print (10 seconds)
const long uploadInterval = 30000;  // Interval for upload (30 seconds)

// Fan speed
int fan1Speed = 0;  // Variable to store Fan 1 speed (PWM value)
int fan2Speed = 0;  // Variable to store Fan 2 speed (PWM value)
int co2ppm    = 0;  //initialize the co2 ppm value globally

WiFiClientSecure client;
// Replace with your Telegram bot token
String botToken = "8191757329:AAHrji16r_-Noj7HlDFLuAoL6tFp1wg9GSk";
#define CHAT_ID "-4558371214"

UniversalTelegramBot bot(botToken, client);
long lastTimeBotChecked = 0;  // To manage Telegram request timing
bool autoFanEnabled = true;   // By default, automatic fan control is enabled
int manualFanSpeed = 0;       // Store the manual fan speed (0-255)

//--------------------------------------SETUP---------------------------------------------//
void setup() {
  Serial.begin(115200); 
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  // WiFi setup
  initWifi();
  
  #ifdef ESP32
    client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  #endif

  // Initialize EEPROM with enough space for both load cell and pH sensor
  EEPROM.begin(256); // Initialize EEPROM once for all calibration data  
  
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
  LoadCell.begin();
  float calibrationValue = LoadCell.getCalFactor(); // Load saved calibration factor
  LoadCell.setCalFactor(calibrationValue); // Set the calibration factor
  LoadCell.start(2000, true); // Stabilize and perform tare
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Tare Timeout, check connections.");
  } else {
    Serial.println("Startup complete. Ready for manual calibration ");
  }

  // pH sensor setup
  ph.begin();

// Set initial fan speed to 0 (fans off)
  pinMode(fan1pin,OUTPUT);
  pinMode(fan2pin,OUTPUT);
  analogWrite(FAN1_PWM_PIN, 0);
  analogWrite(FAN2_PWM_PIN, 0);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check for new data from HX711 regularly
  if (LoadCell.update()) {
    // Load cell is providing new data
  }
  // Check for Telegram messages
  checkTelegramMessages();
  
  // Check if it's time to print to serial (every 10 seconds)
  if (currentMillis - previousMillisSerial >= serialInterval) {
    previousMillisSerial = currentMillis; // Save the last print time
    printSensorDataToSerial(); // Call the function to print sensor data
  }

  // Check if it's time to upload to Google Sheets (every 30 seconds)
  if (currentMillis - previousMillisUpload >= uploadInterval) {
    previousMillisUpload = currentMillis; // Save the last upload time
    uploadDataToGoogleSheets(); // Call the function to upload data to Google Sheets
  }

  // // Receive command from serial terminal
  // if (Serial.available() > 0) {
  //   char inByte = Serial.read();

  //   if (inByte == 't') {
  //     // Tare HX711
  //     LoadCell.tareNoDelay();
  //   } else if (inByte == 'c') {
  //     // Calibrate load cell
  //     calibrateLoadCell();
  //   } else if (inByte == 'enterph') {
  //     // Enter pH calibration mode
  //     enterPHCalibrationMode();
  //   } else if (inByte == 'exitph') {
  //     // Exit pH sensor calibration mode and save
  //     exitPHCalibrationMode();
  //   }
  // }

  // // Perform pH calibration if in calibration mode
  // if (inPHCalibrationMode) {
  //   ph.calibration(voltage, temperature); // Perform pH sensor calibration process
  // }

  // // Check if last tare operation is complete
  // if (LoadCell.getTareStatus() == true) {
  //   Serial.println("Tare complete");
  // }
}

// Function to print sensor data to serial every 10 seconds
void printSensorDataToSerial() {
  Serial.println("Printing sensor data to serial...");

  // Get smoothed value from the dataset and print it
  float weight = LoadCell.getData();
  Serial.print("Load_cell output value: ");
  Serial.println(weight);

  // pH sensor reading
  voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // Read the voltage
  phValue = ph.readPH(voltage, temperature); // Convert voltage to pH with temperature compensation
  Serial.print("pH: ");
  Serial.println(phValue);

  // Read temperature probe
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  Serial.print("Temperature (C): ");
  Serial.println(tempC);

  // Read CO2 sensor
  int co2ppm = mhz19.getCO2PPM();
  Serial.print("CO2 PPM: ");
  Serial.println(co2ppm);

  // Read moisture sensors
  readMoistureSensor();

  // Read DHT sensor
  readDHTSensor();

  // Fan speed control based on temperature
  controlFans(tempC);
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
  output_moisture = measured_moisture/4095 * 100;
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
    Serial.print("Humidity: ");
    Serial.print(dhtHumidity);
    Serial.print("%  Temperature: ");
    Serial.print(dhtTemperature);
    Serial.println("°C");
}

// Fan control based on temperature
void controlFans(float temperature) {
  if (autoFanEnabled) {
    // Automatic fan control based on temperature thresholds
    if (temperature > TEMP_HIGH) {
      fan1Speed = 153;
      fan2Speed = 153;
    } else if (temperature > TEMP_MEDIUM && temperature <= TEMP_HIGH) {
      fan1Speed = 102;
      fan2Speed = 102;
    } else if (temperature > TEMP_LOW && temperature <= TEMP_MEDIUM) {
      fan1Speed = 51;
      fan2Speed = 51;
    } else {
      fan1Speed = 25;
      fan2Speed = 25;
    }
  } else {
    // Manual fan control
    fan1Speed = manualFanSpeed;
    fan2Speed = manualFanSpeed;
  }
  analogWrite(FAN1_PWM_PIN, fan1Speed);
  analogWrite(FAN2_PWM_PIN, fan2Speed);
  Serial.println("Fan speeds set based on current mode.");
}

// // Calibration function for Load Cell
// void calibrateLoadCell() {
//   Serial.println("***");
//   Serial.println("Start calibration:");
//   Serial.println("Place the load cell an a level stable surface.");
//   Serial.println("Remove any load applied to the load cell.");
//   Serial.println("Send 't' from serial monitor to set the tare offset.");

//   boolean _resume = false;
//   while (_resume == false) {
//     LoadCell.update();
//     if (Serial.available() > 0) {
//       if (Serial.available() > 0) {
//         char inByte = Serial.read();
//         if (inByte == 't') LoadCell.tareNoDelay();
//       }
//     }
//     if (LoadCell.getTareStatus() == true) {
//       Serial.println("Tare complete");
//       _resume = true;
//     }
//   }

//   Serial.println("Now, place your known mass on the loadcell.");
//   Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

//   float known_mass = 0;
//   _resume = false;
//   while (_resume == false) {
//     LoadCell.update();
//     if (Serial.available() > 0) {
//       known_mass = Serial.parseFloat();
//       if (known_mass != 0) {
//         Serial.print("Known mass is: ");
//         Serial.println(known_mass);
//         _resume = true;
//       }
//     }
//   }

//   LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
//   float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //get the new calibration value

//   Serial.print("New calibration value has been set to: ");
//   Serial.print(newCalibrationValue);
//   Serial.println(", use this as calibration value (calFactor) in your project sketch.");
//   Serial.print("Save this value to EEPROM adress ");
//   Serial.print(calVal_eepromAdress);
//   Serial.println("? y/n");

//   _resume = false;
//   while (_resume == false) {
//     if (Serial.available() > 0) {
//       char inByte = Serial.read();
//       if (inByte == 'y') {
// #if defined(ESP8266)|| defined(ESP32)
//         EEPROM.begin(512);
// #endif
//         EEPROM.put(calVal_eepromAdress, newCalibrationValue);
// #if defined(ESP8266)|| defined(ESP32)
//         EEPROM.commit();
// #endif
//         EEPROM.get(calVal_eepromAdress, newCalibrationValue);
//         Serial.print("Value ");
//         Serial.print(newCalibrationValue);
//         Serial.print(" saved to EEPROM address: ");
//         Serial.println(calVal_eepromAdress);
//         _resume = true;

//       }
//       else if (inByte == 'n') {
//         Serial.println("Value not saved to EEPROM");
//         _resume = true;
//       }
//     }
//   }

//   Serial.println("End calibration");
//   Serial.println("***");
//   Serial.println("To re-calibrate, send 'r' from serial monitor.");
//   Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
//   Serial.println("***");
// }


// // Enter pH calibration mode
// void enterPHCalibrationMode() {
//   Serial.println("Entering pH calibration mode. Use buffer solutions (4.0 or 7.0).");
//   inPHCalibrationMode = true; // Enable calibration mode
// }

// // Exit pH calibration mode and save data
// void exitPHCalibrationMode() {
//   if (inPHCalibrationMode) {
//     Serial.println("Exiting pH calibration mode and saving data.");
//     inPHCalibrationMode = false; // Disable calibration mode
//   } else {
//     Serial.println("Error: Not in pH calibration mode.");
//   }
// }

void handleTelegramMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    String text = bot.messages[i].text;

    if (text == "/start") {
      String welcomeMsg = "Welcome to the Fan Control Mid!\n"
                          "Commands:\n"
                          "/auto_on_bot - Enable automatic fan control\n"
                          "/auto_off_bot - Disable automatic fan control\n"
                          "/set_speed_bot - Set fan speed manually (0-255)";
      bot.sendMessage(chat_id, welcomeMsg, "");
    } 
    else if (text == "/auto_on_bot") {
      autoFanEnabled = true;
      bot.sendMessage(chat_id, "Automatic fan control enabled.", "");
    } 
    else if (text == "/auto_off_bot") {
      autoFanEnabled = false;
      bot.sendMessage(chat_id, "Automatic fan control disabled. Use /set_speed_bot to manually adjust the fan speed.", "");
    } 
    else if (text.startsWith("/set_speed_bot")) {
      int speed = text.substring(15).toInt();
      if (speed >= 0 && speed <= 255) {
        manualFanSpeed = speed;
        bot.sendMessage(chat_id, "Manual fan speed set to: " + String(manualFanSpeed), "");
      } else {
        bot.sendMessage(chat_id, "Invalid speed. Please enter a number between 0 and 255.", "");
      }
    } 
    else {
      bot.sendMessage(chat_id, "Unknown command. Use /start to see available commands.", "");
    }
  }
}

void checkTelegramMessages() {
  if (millis() - lastTimeBotChecked > 1000) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      handleTelegramMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotChecked = millis();
  }
}

void uploadDataToGoogleSheets() {
  Serial.println("Uploading data to Google Sheets...");

  if (WiFi.status() == WL_CONNECTED) {
    // Create a URL for sending data to Google Sheets
    String sendDataURL = Web_App_URL + "?sts=write";
    sendDataURL += "&temp=" + String(sensors.getTempCByIndex(0)); // Temperature probe
    sendDataURL += "&co2=" + String(mhz19.getCO2PPM()); // CO2 PPM
    sendDataURL += "&pH=" + String(phValue); // pH value
    sendDataURL += "&moisture=" + String(output_moisture); // Soil moisture
    sendDataURL += "&weight=" + String(LoadCell.getData()); // Load cell weight
    sendDataURL += "&humd=" + String(dhtHumidity); // dht22 humidity
    sendDataURL += "&airtemp=" + String(dhtTemperature); // DHT22 temperature
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