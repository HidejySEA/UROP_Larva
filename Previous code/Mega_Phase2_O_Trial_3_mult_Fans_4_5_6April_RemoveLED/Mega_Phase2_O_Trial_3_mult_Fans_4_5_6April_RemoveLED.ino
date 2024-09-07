#include "mq4_sensor.h"
// List of Analog Pins
// A0 Ammonia Reserved
// A1 Methane (In use)
// A2
// A3 Moisture (In use)
// A4 Moisture (In use)
// A5 Moisture (In use)
// A6 Moisture Reserved
// A7 Moisture Reserved
// A8 Moisture Reserved
// A9
// A10
// A11
// A12
// A13 pH Sensor Reserved
// A14 pH Sensor Reserved
// A15 pH Sensor (In Use)

//#define INTERVAL 7000 // 7 Seconds
//#define INTERVAL 10000 // 10 Seconds
//#define INTERVAL 60000 // 1 minute
#define INTERVAL 600000 // 10 minutes


// Start of Threshold Values
float min_humi = 40.0; // RH(%) at 40% has Mortality of 26%
float min_acceptable_humi = 20; // humidity below this will not be considered for regulation of fan
float max_acceptable_humi = 100; // humidity above this will not be considered for regulation of fan

float lower_limit_temp = 33;
float upper_limit_temp = 35;

float lower_limit_temp_sub = 36;
float upper_limit_temp_sub = 45;

float min_acceptable_temp = 20; // temperature below this will not be considered for regulation of fan
float max_acceptable_temp = 50; // temperature above this will not be considered for regulation of fan
// End of Threshold Values


// Start SD Card Module
#include <SPI.h>
#include <SD.h>
#include <Wire.h> // Initialize the I2C bus
// change this to match your SD shield or module;
const int chipSelect = 53; //cs | SS pin from the sd Card Module is connected to 53. (Mega Hardware SPI Pins)
File dataFile;
// End SD Card Module


// Start DS18B20 (Temperature)
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 22 on the Arduino
#define ONE_WIRE_BUS 22

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Addresses of 2 DS18B20s
uint8_t sensorA1[8] = { 0x28, 0x71, 0x9E, 0xAC, 0x70, 0x22, 0x08, 0x62 };// Internal Air Temp #1 located at Inlet
uint8_t sensorA2[8] = { 0x28, 0x1F, 0xA9, 0x3C, 0x70, 0x22, 0x09, 0xD8 };// Internal Air Temp #2 located at Outlet

int myTemp_Address[] = {sensorA1, sensorA2};
const int num_of_AT_sensors = sizeof(myTemp_Address) / sizeof(myTemp_Address[0]);
float measured_Temp_Array[num_of_AT_sensors]; // Store an instance of all air temperature measured

// Addresses of 3 DS18B20s (Waterproof)
uint8_t sensor1[8] = { 0x28, 0x92, 0xCE, 0x57, 0x04, 0xE1, 0x3D, 0x73 };// Temp in Substrate #1 located at the bottom container
uint8_t sensor2[8] = { 0x28, 0x79, 0x51, 0x43, 0xD4, 0xE1, 0x3C, 0x0B };// Temp in Substrate #2 located in middle container
uint8_t sensor3[8] = { 0x28, 0xFE, 0x82, 0x43, 0xD4, 0xE1, 0x3C, 0x74 };// Temp in Substrate #3 located at the Top container

int myTemp_sub_Address[] = {sensor1, sensor2, sensor3};

const int num_of_temp_sub_sensors = sizeof(myTemp_sub_Address) / sizeof(myTemp_sub_Address[0]);
float measured_Temp_Sub_Array[num_of_temp_sub_sensors]; // Store an instance of all substrate temperature measured
// End DS18B20 (Temperature)


// Start Capacitive Soil Moisture Sensor
// Define analog input
#define m_sensorPin1 A3
#define m_sensorPin2 A4
#define m_sensorPin3 A5

byte analog_pinsOut_moisture[] = {m_sensorPin1, m_sensorPin2, m_sensorPin3};

const int num_of_msensors = sizeof(analog_pinsOut_moisture) / sizeof(analog_pinsOut_moisture[0]);
float measured_moisture_Array[num_of_msensors]; // Store an instance of all moisture measured
// End of Capacitive Soil Moisture Sensor


// Start Gravity Analog pH Sensors
#include "DFRobot_PH.h"
#include <EEPROM.h>

float voltage, phValue, phtemperature = 25;
DFRobot_PH ph;

// Define analog input
#define pH_sensorPin3 A15

byte analog_pinsOut_ph[] = {pH_sensorPin3};

const int num_of_pHsensors = sizeof(analog_pinsOut_ph) / sizeof(analog_pinsOut_ph[0]);
float measured_pH_sub_Array[num_of_pHsensors]; // Store an instance of all pH measured
// End Gravity Analog pH Sensors


// Start MQ4 (Methane)
// Go to mq4_sensor.h file to select pin
const int num_of_ch4_gas_sensors = 1; // Manually Keyed in
float measured_ch4_Array[num_of_ch4_gas_sensors]; // Store an instance of all methane measured
// End MQ4 (Methane)


// Start DHT22
#include "DHT.h"
#define DHT2PIN 29     // Digital pin connected to the DHT sensor #2 located at the Outlet

#define DHTTYPE DHT22   // DHT 22 

// Initialize each DHT sensor.
DHT dht_outlet(DHT2PIN, DHTTYPE);

byte digital_pinsOut_humidity[] = {DHT2PIN};

const int num_of_humidity_sensors = 1; // Manually Keyed in
float measured_humidity_Array[num_of_humidity_sensors]; // Store an instance of all humidity measured
// End DHT22


// Start of 4-wire Fan
float temperature;
const int fan_control_pin1 = 4;
const int fan_control_pin2 = 5;
const int fan_control_pin3 = 6;
const int fan_control_pin4 = 7;
const int fan_control_pin5 = 8;
const int fan_control_pin6 = 9;

int digital_pinsOut_fan[] = {fan_control_pin1, fan_control_pin2, fan_control_pin3, fan_control_pin4, fan_control_pin5, fan_control_pin6};
const int num_of_fans = sizeof(digital_pinsOut_fan) / sizeof(digital_pinsOut_fan[0]);

float measured_fan_lvl_Array[1]; // Manually Keyed in

// Duty cycle: 0 (Turn Off) and 255 (Maximum)
int lowest_speed_pwm = 25; // 10%
int norm_speed_pwm = 51; // 20%
int intermediate_speed_pwm = 102; // 40% 
int high_speed_pwm = 153; // 60%
// End of 4-wire Fan


int error_open_file_led = 30;

uint32_t mega_Millis = 0;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  Serial.println("Initializing SD card...");
  // see if the SD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    Serial.println("Card failed, Please Check if Card is present");
    digitalWrite(error_open_file_led, HIGH); // turn LED on
    // don't do anything more:
    return;
  }
  Serial.println("Initialization Successful!");
  Serial.println("Card Found.");

  // Open the File
  dataFile = SD.open("datalog.txt", FILE_WRITE); // DO NOT CHANGE THE NAME, SD card library does not support long filenames (8.3 filename format)
  dataFile.println("");
  dataFile.println("Start of Data Logging"); // Print Header information to file
  dataFile.println("Line Break"); // Print Header information to file
  dataFile.close(); // close the File


  //~ Start DS18B20
  sensors.begin();  // Start up the library
  Serial.println("DS18B20 Begin");
  //~ End DS18B20


  //~~~ Start Gravity Analog pH Sensors
  ph.begin();  // Start up the library
  Serial.println("Ph Begin");
  //~~~ End Gravity Analog pH Sensors


  //~~~~ Start MQ4 (Methane)
  setup_mq4();
  //~~~~ End MQ4 (Methane)


  //~~~~~ Start DHT Sensors
  dht_outlet.begin();
  //~~~~~ End DHT Sensors


  //~~~~~~ Start 4-wire Fan
  initialize_all_fans();
  //~~~~~~ End 4-wire Fan

  pinMode(error_open_file_led, OUTPUT);    // sets the digital pin as output
  digitalWrite(error_open_file_led, LOW); // turn LED for Normal Speed Off by making the voltage level low
}

void loop() {

  int result;

  if ( (millis() - mega_Millis) >= INTERVAL )
  {
    Send1(); // Get and Store Air Temp readings into an array
    Send2(); // Get and Store Substrate Temp readings into an array
    Send3(); // Get and Store Moisture readings into an array
    Send4(); // Get and Store PH readings into an array
    Send5(); // Get and Store Methane readings into an array
    Send6(); // Get and Store Relative Humidity readings into an array

    result = check_threshold();
    measured_fan_lvl_Array[0] = result;

    savetoSDCard();

    optimization(result); // Regulate Fan Speed

    mega_Millis = millis();
  }
}

void initialize_all_fans() {
  // Occurs only in the setup()
  for (int i = 0; i < num_of_fans; i++) {
    pinMode(digital_pinsOut_fan[i], OUTPUT); // set all fans as output
    analogWrite(digital_pinsOut_fan[i], 0); // // Initialize the duty cycle for each fan to 0: 0 (always off) and 255 (always on)
  }
}

void Send1() // Air Temp
{
  sensors.requestTemperatures();// Send the command to instruct all sensors on the bus to perform a temperature conversion.

  for (int i = 0; i < num_of_AT_sensors; i++) {
    measured_Temp_Array[i] = getTemperature(myTemp_Address[i]);
  }
}

void Send2() // Substrate Temp
{
  for (int i = 0; i < num_of_temp_sub_sensors; i++) {
    measured_Temp_Sub_Array[i] = getTemperature(myTemp_sub_Address[i]);
  }
}

float getTemperature(DeviceAddress deviceAddress)
{
  float mytempC = sensors.getTempC(deviceAddress);
  return float(mytempC);
}

void Send3() // Moisture
{
  for (int i = 0; i < num_of_msensors; i++) {
    measured_moisture_Array[i] = analogRead(analog_pinsOut_moisture[i]);
  }
}

void Send4() // pH
{
  for (int i = 0; i < num_of_pHsensors; i++) {
    measured_pH_sub_Array[i] = getpH(analog_pinsOut_ph[i]);
  }
}

float getpH(byte Sensor)
{
  //temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
  voltage = analogRead(Sensor) / 1024.0 * 5000; // read the voltage
  phValue = ph.readPH(voltage, phtemperature); // convert voltage to pH with temperature compensation

  ph.calibration(voltage, phtemperature);          // calibration process by Serail CMD
  return phValue;
}


void Send5() // Methane
{
  for (int i = 0; i < num_of_ch4_gas_sensors; i++) {
    measured_ch4_Array[i] = read_mq4();
  }
}

void Send6() // Relative humidity
{
  float myReading = 0;
  myReading = dht_outlet.readHumidity();
  if (isnan(myReading)) {
    Serial.println(F("Failed to read from DHT sensor at outlet!"));
    return;
  }
  measured_humidity_Array[0] = myReading;
}



void savetoSDCard() {

  // Open the file.
  dataFile = SD.open("datalog.txt", FILE_WRITE); // DO NOT CHANGE THE NAME, SD card library does not support long filenames (8.3 filename format)

  Serial.println("Opening datalog file!"); // Indicator Opening File in SD Card on Serial Monitor
  // if the file is available, write to it:
  // log the readings
  if (dataFile) {
    dataFile.print("Internal_Air_Temp(degree celsius); "); // AVOID using : colon as delimited

    for (int i = 0; i < num_of_AT_sensors; i++) {
      float myTemp = 0;
      myTemp = measured_Temp_Array[i];
      Serial.print("Measured Temp Readings: ");
      Serial.println(myTemp); // Display Data on Serial Monitor
      dataFile.print(myTemp); // Print Data into file
      dataFile.print(";   "); // Print Semi-colon as delimited into file
    }
    dataFile.print("|"); // Indicate the Next Sensors data logging Sequence into file


    dataFile.print("Temp_substrate(degree celsius); "); // AVOID using : colon as delimited

    for (int i = 0; i < num_of_temp_sub_sensors; i++) {
      float myTemp = 0;
      myTemp = measured_Temp_Sub_Array[i];
      Serial.print("Measured Temp in Substrate: ");
      Serial.println(myTemp); // Display Data on Serial Monitor
      dataFile.print(myTemp); // Print Data into file
      dataFile.print(";   "); // Print Semi-colon as delimited into file
    }
    dataFile.print("|"); // Indicate the Next Sensors data logging Sequence into file


    dataFile.print("Moisture (AnalogRead); "); // AVOID using : colon as delimited

    for (int i = 0; i < num_of_msensors; i++) {
      float myreading = 0;
      myreading = measured_moisture_Array[i];
      Serial.print("Measured Moisture Readings (AnalogRead): ");
      Serial.println(myreading);
      dataFile.print(myreading); // Print Data into file
      dataFile.print(";   "); // Print Semi-colon as delimited into file
    }
    dataFile.print("|"); // // Indicate the Next Sensors data logging Sequence into file


    dataFile.print("Actual pH; "); // AVOID using : colon as delimited

    for (int i = 0; i < num_of_pHsensors; i++) {
      float measured_pH = 0;
      measured_pH = measured_pH_sub_Array[i];
      Serial.print("Measured pH Readings: ");
      Serial.println(measured_pH);
      //Serial.println(measured_pH, 1); //optional second parameter specifies the base (format) to use 1 decimal place
      dataFile.print(measured_pH); // Print Data into file
      dataFile.print(";   "); // Print Semi-colon as delimited into file
    }
    dataFile.print("|"); // Indicate the Next Sensors data logging Sequence into file


    dataFile.print("Methane(ppm); "); // AVOID using : colon as delimited

    for (int i = 0; i < num_of_ch4_gas_sensors; i++) {
      float myreading = 0;
      myreading = measured_ch4_Array[i];
      Serial.print("Methane PPM Readings: ");
      Serial.println(myreading);

      dataFile.print(myreading); // Print Data into file
      dataFile.print(";   "); // Print Semi-colon as delimited into file
    }
    dataFile.print("|"); // Indicate the Next Sensors data logging Sequence into file


    dataFile.print("DHT22(%); "); // AVOID using : colon as delimited

    for (int i = 0; i < num_of_humidity_sensors; i++) {
      float my_humival = 0;
      my_humival = measured_humidity_Array[i];
      Serial.print("Measured Humidity(%): ");
      Serial.println(my_humival);
      dataFile.print(my_humival); // Print Data into file
      dataFile.print(";   "); // Print Semi-colon as delimited into file
    }

    dataFile.print("|"); // // Indicate the Next Sensors data logging Sequence into file




    dataFile.print("Fan Level; "); // AVOID using : colon as delimited

    for (int i = 0; i < 1; i++) {
      float myfanlvl = 0;
      myfanlvl = measured_fan_lvl_Array[i];
      Serial.print("Fan level: ");
      Serial.println(myfanlvl);
      dataFile.print(myfanlvl); // Print Data into file
      dataFile.print(";   "); // Print Semi-colon as delimited into file
    }
    dataFile.print("|"); // // Indicate the Next Sensors data logging Sequence into file


    dataFile.println(""); // Move Cursor to Next line

    Serial.println("data stored"); // Indicator the end of the data logging process on Serial Monitor

    Serial.println("Closing datalog file!"); // Indicator Closing File in SD Card on Serial Monitor
    Serial.println(""); // Move cursor to the next line
    dataFile.close(); // close the File

  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening datalog.txt");
    digitalWrite(error_open_file_led, HIGH); // turn LED on
  }
}


int check_threshold() {
  int error_code = 0;
  int no_error = 0;

  if (error_code == no_error) {
    // ~ Check Temperature (Substrate) Readings
    for (int i = 0; i < num_of_temp_sub_sensors; i++) {
      float myTemp_sub = 0;
      myTemp_sub = measured_Temp_Sub_Array[i];

      if ((myTemp_sub >= min_acceptable_temp) && (myTemp_sub <= max_acceptable_temp)) {

        if ((myTemp_sub >= lower_limit_temp_sub) && (myTemp_sub <= upper_limit_temp_sub)) {
          error_code = 2;
          break;
        }
        else if (myTemp_sub >= upper_limit_temp_sub) {
          error_code = 3;
          break;
        } else {
          error_code = no_error;
          Serial.println("Temperature in Substrate within Acceptable Limits");
        }
      } else {
        Serial.println("A Reading found to be Invalid");
      }
    }

  } else {
    Serial.println("Error Code detected, Skipped Checking Temp in Substrate");
  }

  if (error_code == no_error) {
    // ~ Check Air Temp Readings
    for (int i = 0; i < num_of_AT_sensors; i++) {
      float myTemp = 0;
      myTemp = measured_Temp_Array[i];

      if ((myTemp >= min_acceptable_temp) && (myTemp <= max_acceptable_temp)) {

        if ((myTemp >= lower_limit_temp) && (myTemp <= upper_limit_temp)) {
          error_code = 2;
          break;
        }
        else if (myTemp >= upper_limit_temp) {
          error_code = 3;
          break;
        } else {
          error_code = no_error;
          Serial.println("Air Temperature within Acceptable Limits");
        }
      } else {
        Serial.println("A Reading found to be Invalid");
      }
    }

  } else {
    Serial.println("Error Code detected, Skipped Checking Air Temp");
  }

  if (error_code == no_error) {
    // ~ Check Relative Humidity Readings Readings
    for (int i = 0; i < num_of_humidity_sensors; i++) {
      float myHumi = 0;
      myHumi = measured_humidity_Array[i];

      if ((myHumi >= min_acceptable_humi) && (myHumi <= max_acceptable_humi)) {

        if ((myHumi <= min_humi)  ) {
          error_code = 1;
          break;
        }
        else {
          error_code = no_error;
          Serial.println("Humidity within Acceptable Limits");
        }

      } else {
        Serial.println("A Reading found to be Invalid");
      }
    }

  } else {
    Serial.println("Error Code detected, Skipped Checking Humidity");
  }

  Serial.println("END of checking threshold limits");
  Serial.println("");
  Serial.println("");
  return error_code;
}


void pwm_response(int my_speed) {
  for (int i = 0; i < num_of_fans; i++) {
    analogWrite(digital_pinsOut_fan[i], my_speed); // // Initialize the duty cycle for each fan to 0: 0 (always off) and 255 (always on)
  }
}

void optimization(int error_code) {
  switch (error_code) {
    case 1:
      pwm_response(lowest_speed_pwm); // Change Duty Cycle to reduce Fan Speed to lowest setting
      break;
    case 2:
      pwm_response(intermediate_speed_pwm); // Change Duty Cycle to increase Fan Speed to Intermediate
      break;
    case 3:
      pwm_response(high_speed_pwm); // Change Duty Cycle to increase Fan Speed to High
      break;
    default:
      pwm_response(norm_speed_pwm); // Duty Cycle set at default, Regular Operating Fan speed
      break;
  }
}
