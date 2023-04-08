#include <Arduino.h>
#include <Wire.h>


uint8_t SDApin = SDA;//33
uint8_t SCLpin = SCL;//35

void setup() {
  // put your setup code here, to run once:
  Wire.begin(SDApin,SCLpin);

  Serial.begin(115200);

  Serial.println("\nI2C Scanner");

}

void loop() {
  // put your main code here, to run repeatedly:
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);  
}