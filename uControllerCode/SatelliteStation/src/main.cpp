#include <Arduino.h>
#include <WiFi.h>

//Declare Variables

//Wifi Login
const char* serverssid     = "test_Wifi";
const char* serverpassword = "testMyWifi";

//Timer variables
int timePeriod = 1000;
unsigned long currentTime = 0;

//Declare functions
void onCloseContact();


void setup() {
  // put your setup code here, to run once:
  onCloseContact();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();
      while(millis() < currentTime + timePeriod){
        //wait approx. [period] ms
    }
    if(WiFi.status() == WL_CONNECTED){
        Serial.println("Connected");
    }
  
}

void onCloseContact() {
    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(serverssid, serverpassword);

        while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
}

