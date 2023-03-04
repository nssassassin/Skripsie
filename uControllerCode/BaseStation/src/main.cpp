#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>

const char* ssid = "router_ssid";
const char* password = "router_wifi_password";
const char* apSSID = "base_station_ssid";
const char* apPassword = "base_station_password";

const int chipSelect = 5; // change this to the chip select pin of your SD card reader
const uint16_t port = 8888;

WebServer server(port);
IPAddress Ip(192, 168, 2, 100);
IPAddress NMask(255, 255, 255, 0);

void setup() {
  Serial.begin(115200);

  // Configure Wi-Fi hotspot
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apSSID, apPassword);
  WiFi.softAPConfig(Ip, Ip, NMask);
  Serial.println("Wi-Fi hotspot started");
    Serial.print("[+] AP Created with IP Gateway ");
    Serial.println(WiFi.softAPIP()); 

  //Only wait for this connection to check server later.
  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  Serial.println("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("Connected to Wi-Fi");
  


  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("Card initialized.");

  // Route for receiving data
  server.on("/", HTTP_POST, [](){
    String data = server.arg("data");
    Serial.println(data);

    // Open CSV file for writing
    File file = SD.open("data.csv", FILE_WRITE);
    if (!file) {
      Serial.println("File open failed");
      server.send(500, "text/plain", "File open failed");
      return;
    }

    // Write data to file
    file.println(data);
    file.close();
    
    server.send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("Server started");
}

void loop() {
  // Check if Wi-Fi network is still connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi network disconnected");
    
    // Attempt to reconnect to Wi-Fi network
    WiFi.begin(ssid, password);
    Serial.println("Connecting to Wi-Fi...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting...");
    }
    Serial.println("Connected to Wi-Fi");
  }
  
  server.handleClient();
}

//Still need to handle write to server or api, granted access is given to wifi.