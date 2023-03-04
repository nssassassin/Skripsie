#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <SD.h>

const char* ssid = "base_station_ssid";
const char* password = "base_station_password";

const IPAddress serverIP(192, 168, 2, 100); // replace with the IP address of the base station
const uint16_t serverPort = 8888; // replace with the port number of the base station

WiFiClient client;

const int chipSelect = 5; // change this to the chip select pin of your SD card reader

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi network
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("Connected to WiFi");

  // Connect to base station
  Serial.println("Connecting to base station...");
  while (!client.connect(serverIP, serverPort)) {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("Connected to base station");

  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("Card initialized.");
}

void loop() {
  // Open CSV file for reading
  File file = SD.open("data.csv");
  if (!file) {
    Serial.println("File not found");
    return;
  }

  // Read CSV values and send them to base station
  while (file.available()) {
    String line = file.readStringUntil('\n');
    client.write(line.c_str(), line.length());
    Serial.println(line);
  }
  file.close();

  // Wait for response from base station
  while (client.available()) {
    String response = client.readString();
    Serial.println(response);
  }

  delay(1000);
}
