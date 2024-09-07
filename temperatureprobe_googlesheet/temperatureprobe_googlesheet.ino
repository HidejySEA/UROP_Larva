#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const char* ssid     = "notyouriphone";
const char* password = "hidejy123";

const char* resource = "https://maker.ifttt.com/trigger/temp_data/with/key/dhQ0R8oARgUE5M9wfjWahi";
const char* server = "maker.ifttt.com";

#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

unsigned long previousMillis = 0; 
const long interval = 60000; // interval to wait for (milliseconds) - 1 minute

void setup() {
  Serial.begin(115200); 
  delay(2000);
  pinMode(2, INPUT_PULLUP);

  sensors.begin();
  initWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  
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

  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // Temperature in Celsius
  String jsonObject = String("{\"value1\":\"") + tempC + "\",\"value2\":\"" + ""
                      + "\",\"value3\":\"" + "" + "\"}";
                   
  client.println(String("POST ") + resource + " HTTP/1.1");
  client.println(String("Host: ") + server); 
  client.println("Connection: close\r\nContent-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonObject.length());
  client.println();
  client.println(jsonObject);
       
  int timeout = 5 * 10; // 5 seconds             
  while(!client.available() && (timeout-- > 0)){
    delay(100);
  }
  if(!client.available()) {
    Serial.println("No response...");
  }
  while(client.available()){
    Serial.write(client.read());
  }
  
  Serial.println("\nclosing connection");
  client.stop(); 
}
