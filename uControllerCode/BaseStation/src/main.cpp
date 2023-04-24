#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <K30_I2C.h>
#include <SensirionI2CSen5x.h>
#include <WiFi.h>
#include <esp_now.h>


#define RXD2 16
#define TXD2 17


//This is for the sensirion
#define MAXBUF_REQUIREMENT 48
#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

uint8_t SDApin = 8;//33
uint8_t SCLpin = 9;//35
SensirionI2CSen5x sen5x;


//This is the part for the k30 sensor init
#define sdak30 8
#define sclk30 9
#define K30_I2C_ADDRESS 0x68 // 7-bit address of K30 sensor
K30_I2C k30(K30_I2C_ADDRESS, sdak30, sclk30); // Create a K30 object


//This is for esp-now
//48:27:E2:46:FD:4A This is the base station address.
//48:27:E2:46:FE:12 This is the satellite address
uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x46, 0xFD, 0x4A};
//uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x46, 0xFE, 0x12};
String success;

typedef struct struct_message {
    float co2_;
    float massConcentrationPm1p0_;
    float massConcentrationPm2p5_;
    float massConcentrationPm4p0_;
    float massConcentrationPm10p0_;
    float ambientHumidity_;
    float ambientTemperature_;
    float vocIndex_;
    float noxIndex_;   
} struct_message;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
   memcpy(&receivedata, incomingData, sizeof(receivedata));
   Serial.print("Bytes received: ");
  Serial.println(len);
   Serial.println(receivedata.ambientTemperature_);

 }



//end

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    delay(3000);
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);
    delay(3000);
  Serial.println("Start up");  


  //sensirion
  sen5x.begin(Wire);

  uint16_t error;
  char errorMessage[256];
  error = sen5x.deviceReset();
  // while (error) {
  //     Serial.print("Error trying to execute deviceReset(): ");
  //     errorToString(error, errorMessage, 256);
  //     Serial.println(errorMessage);
  //     delay(1000);
  //     error = sen5x.deviceReset();
  // }
  //     // Start Measurement
  //   error = sen5x.startMeasurement();
  //   if (error) {
  //       Serial.print("Error trying to execute startMeasurement(): ");
  //       errorToString(error, errorMessage, 256);
  //       Serial.println(errorMessage);
  //   }
  

  //ESP NOW
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
    // Add peer        
  while (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

}


//How to send data
struct_message sendingdata;
struct_message receivedata;
//sendingdata.co2_ = co2;
//esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendingdata, sizeof(sendingdata));



void loop() {
  // put your main code here, to run repeatedly:

  //k30
//   int co2; // Variable to store CO2 value
//   int status; // Variable to store status of reading
//   status = k30.readCO2(co2); // Read CO2 value from sensor
//   if (status == 0) { // If reading was successful
//     Serial.print("CO2:"); // Print CO2 label
//     Serial.print(co2); // Print CO2 value
//     //Serial.print(" ppm"); // Print ppm unit
//     Serial.print("\t");
//   } else { // If reading failed
//     Serial.print("Error: "); // Print error label
//     Serial.println(status, HEX); // Print error code in hexadecimal
//   }

// // sensirion
//     uint16_t error;
//     char errorMessage[256];
//     float massConcentrationPm1p0;
//     float massConcentrationPm2p5;
//     float massConcentrationPm4p0;
//     float massConcentrationPm10p0;
//     float ambientHumidity;
//     float ambientTemperature;
//     float vocIndex;
//     float noxIndex;   
//     error = sen5x.readMeasuredValues(
//         massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
//         massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
//         noxIndex);

//     if (error) {
//         Serial.print("Error trying to execute readMeasuredValues(): ");
//         errorToString(error, errorMessage, 256);
//         Serial.println(errorMessage);
//     } else {
//         Serial.print("MassConcentrationPm1p0:");
//         Serial.print(massConcentrationPm1p0);
//         Serial.print("\t");
//         Serial.print("MassConcentrationPm2p5:");
//         Serial.print(massConcentrationPm2p5);
//         Serial.print("\t");
//         Serial.print("MassConcentrationPm4p0:");
//         Serial.print(massConcentrationPm4p0);
//         Serial.print("\t");
//         Serial.print("MassConcentrationPm10p0:");
//         Serial.print(massConcentrationPm10p0);
//         Serial.print("\t");
//         Serial.print("AmbientHumidity:");
//         if (isnan(ambientHumidity)) {
//             Serial.print("n/a");
//         } else {
//             Serial.print(ambientHumidity);
//         }
//         Serial.print("\t");
//         Serial.print("AmbientTemperature:");
//         if (isnan(ambientTemperature)) {
//             Serial.print("n/a");
//         } else {
//             Serial.print(ambientTemperature);
//         }
//         Serial.print("\t");
//         Serial.print("VocIndex:");
//         if (isnan(vocIndex)) {
//             Serial.print("n/a");
//         } else {
//             Serial.print(vocIndex);
//         }
//         Serial.print("\t");
//         Serial.print("NoxIndex:");
//         if (isnan(noxIndex)) {
//             Serial.println("n/a");
//         } else {
//             Serial.println(noxIndex);
//         }
//     }    


  delay(10000); // Wait for 1 second
}