#include <Arduino.h>
#include <WiFi.h>

//Declare Variables

//Wifi Login
const char* apSsid     = "test_Wifi";
const char* apPassword = "testMyWifi";

//Timer variables
int timePeriod = 1000;
unsigned long currentTime = 0;



void setup()
{
    Serial.begin(115200);
    Serial.println("\n[*] Creating AP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSsid, apPassword);
    Serial.print("[+] AP Created with IP Gateway ");
    Serial.println(WiFi.softAPIP());
}

void loop() {

currentTime = millis();
   
  Serial.println("Salig");
   
    while(millis() < currentTime + timePeriod){
        //wait approx. [period] ms
    }

}