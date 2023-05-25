#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <K30_I2C.h>
#include <SensirionI2CSen5x.h>
#include <WiFi.h>
#include <esp_now.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Replace with your network credentials
const char* ssid     = "Deezwhat";
const char* password = "deeznuts";

AsyncWebServer server(80);
AsyncEventSource events("/events");

#include <time.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>






#define SS 34
#define MOSI 35
#define MISO 37
#define SCK 36

#define RXD2 16
#define TXD2 17



WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;


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
//48:27:E2:46:FC:EC This is the base station address.
//48:27:E2:46:FE:12 This is the satellite address
//uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x46, 0xFC, 0xEC};
uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x46, 0xFE, 0x12};
String success;

typedef struct struct_message {
    int Second;
    int Minute;
    int Hour;
    int Day;
    int Month;
    int Year;
    double lat_;
    double lon_;
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
struct_message sendingdata;
struct_message receivedata;
esp_now_peer_info_t peerInfo;

struct_message lastMessage;

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
int RECEIVED = 0;
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
   memcpy(&receivedata, incomingData, sizeof(receivedata));
   Serial.print("Bytes received: ");
  Serial.println(len);
  RECEIVED = 1;
   //Serial.println(receivedata.ambientTemperature_);


}

void initSDCard(){
  if (!SD.begin(SS,SPI,80000000)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(7200);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/index.html", "text/html");
  });

  server.serveStatic("/", SD, "/");

  server.begin();
}



// Save reading number on RTC memory
RTC_DATA_ATTR int readingID = 0;

String dataMessage;
// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}
void logSDCard(    float massConcentrationPm1p0,float massConcentrationPm2p5,float massConcentrationPm4p0,float massConcentrationPm10p0,float ambientHumidity,float ambientTemperature,float vocIndex,float noxIndex, float co2 ) {
  dataMessage = String(readingID) + "," + String(massConcentrationPm1p0) + "," + String(massConcentrationPm2p5) + "," + String(massConcentrationPm4p0) + "," + String(massConcentrationPm10p0) + "," + String(ambientHumidity) + "," + String(ambientTemperature) + "," + String(vocIndex)+ "," + String(noxIndex) + "," + 
                String(co2) + "\r\n";

  //IN ORDER
   // float massConcentrationPm1p0;
   // float massConcentrationPm2p5;
   // float massConcentrationPm4p0;
   // float massConcentrationPm10p0;
   // float ambientHumidity;
   // float ambientTemperature;
   // float vocIndex;
   // float noxIndex;  
   //float co2
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/data.csv", dataMessage.c_str());
}
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
String getFormattedDate(){
time_t epochTime = timeClient.getEpochTime();
struct tm *ptm = gmtime ((time_t *)&epochTime);
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon+1;
  int currentYear = ptm->tm_year+1900;
String date = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay)+ " ";
String formattime =  timeClient.getFormattedTime();
String myTime = date + formattime;
return myTime;
}
void getTimeStamp() {
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = getFormattedDate();
  Serial.println(formattedDate);

  // Extract date
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  Serial.println(dayStamp);
  // Extract time
  timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);
  Serial.println(timeStamp);
}

//end

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    delay(3000);
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);
    delay(3000);
  Serial.println("Start up");  

  SPI.begin(SCK, MISO, MOSI, SS);
  
  //sensirion
  sen5x.begin(Wire);

  uint16_t error;
  char errorMessage[256];
  error = sen5x.deviceReset();
   while (error) {
       Serial.print("Error trying to execute deviceReset(): ");
       errorToString(error, errorMessage, 256);
       Serial.println(errorMessage);
       delay(1000);
       error = sen5x.deviceReset();
   }
       // Start Measurement
     error = sen5x.startMeasurement();
    if (error) {
         Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
         Serial.println(errorMessage);
     }
  

  //ESP NOW
  WiFi.mode(WIFI_AP_STA);
    // Set device as a Wi-Fi Station
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
  }
  initSDCard();

   


    // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
    esp_now_register_send_cb(OnDataSent);
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

//sendingdata.co2_ = co2;
//esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendingdata, sizeof(sendingdata));



void loop() {
  // put your main code here, to run repeatedly:
  if(RECEIVED == 1){
  Serial.print("SATELLITE time: ");
  Serial.print(receivedata.Year);
  Serial.print("/");
  Serial.print(receivedata.Month);
  Serial.print("/");
  Serial.print(receivedata.Day); 
  Serial.print("-");
  Serial.print(receivedata.Hour);
  Serial.print(":");
  Serial.print(receivedata.Minute);
  Serial.print(":");
  Serial.print(receivedata.Second); 
  Serial.print(" ");
  Serial.print("Location: ");
  Serial.print(receivedata.lat_, 10);
  Serial.print(", ");
  Serial.print(receivedata.lon_, 10);
  Serial.print(" ");
  Serial.print("CO2:  +");
  Serial.print(receivedata.co2_);
  Serial.print(","); 
  Serial.print("MassConcentrationPm1p0:  +");
  Serial.print(receivedata.massConcentrationPm1p0_);
  Serial.print(",");
  Serial.print("MassConcentrationPm2p5: ");
  Serial.print(receivedata.massConcentrationPm2p5_);
  Serial.print(",");
  Serial.print("MassConcentrationPm4p0: ");
  Serial.print(receivedata.massConcentrationPm4p0_);
  Serial.print(",");
  Serial.print("MassConcentrationPm10p0: ");
  Serial.print(receivedata.massConcentrationPm10p0_);
  Serial.print(",");
  Serial.print("AmbientHumidity: ");
  if (isnan(receivedata.ambientHumidity_)) {
      Serial.print("n/a");
  } else {
      Serial.print(receivedata.ambientHumidity_);
  }
  Serial.print(",");
   Serial.print("AmbientTemperature: ");
  if (isnan(receivedata.ambientTemperature_)) {
      Serial.print("n/a");
  } else {
      Serial.print(receivedata.ambientTemperature_);
  }
  Serial.print("C,");
  Serial.print("VocIndex: ");
  if (isnan(receivedata.vocIndex_)) {
      Serial.print("n/a");
  } else {
      Serial.print(receivedata.vocIndex_);
  }
  Serial.print(",");
  Serial.print("NoxIndex: ");
  if (isnan(receivedata.noxIndex_)) {
      Serial.println("n/a");
  } else {
      Serial.println(receivedata.noxIndex_);
  }
  RECEIVED = 0;
  }
  //k30
   int co2; // Variable to store CO2 value
   int status; // Variable to store status of reading
   status = k30.readCO2(co2); // Read CO2 value from sensor
   if (status == 0) { // If reading was successful
     //Serial.print("CO2:"); // Print CO2 label
     //Serial.print(co2); // Print CO2 value
//     //Serial.print(" ppm"); // Print ppm unit
     Serial.print("\t");
   } else { // If reading failed
     Serial.print("Error: "); // Print error label
     Serial.println(status, HEX); // Print error code in hexadecimal
   }

// sensirion
    uint16_t error;
    char errorMessage[256];
    float massConcentrationPm1p0;
    float massConcentrationPm2p5;
    float massConcentrationPm4p0;
    float massConcentrationPm10p0;
    float ambientHumidity;
    float ambientTemperature;
    float vocIndex;
    float noxIndex;   
    error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);

    if (error) {
        Serial.print("Error trying to execute readMeasuredValues(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
/*         Serial.print("MassConcentrationPm1p0:");
        Serial.print(massConcentrationPm1p0);
        Serial.print("\t");
        Serial.print("MassConcentrationPm2p5:");
        Serial.print(massConcentrationPm2p5);
        Serial.print("\t");
        Serial.print("MassConcentrationPm4p0:");
        Serial.print(massConcentrationPm4p0);
        Serial.print("\t");
        Serial.print("MassConcentrationPm10p0:");
        Serial.print(massConcentrationPm10p0);
        Serial.print("\t");
        Serial.print("AmbientHumidity:");
        if (isnan(ambientHumidity)) {
            Serial.print("n/a");
        } else {
            Serial.print(ambientHumidity);
        }
        Serial.print("\t");
        Serial.print("AmbientTemperature:");
        if (isnan(ambientTemperature)) {
            Serial.print("n/a");
        } else {
            Serial.print(ambientTemperature);
        }
        Serial.print("\t");
        Serial.print("VocIndex:");
        if (isnan(vocIndex)) {
            Serial.print("n/a");
        } else {
            Serial.print(vocIndex);
        }
        Serial.print("\t");
        Serial.print("NoxIndex:");
        if (isnan(noxIndex)) {
            Serial.println("n/a");
        } else {
            Serial.println(noxIndex);
        } */
    }    


  delay(100); // Wait for 1 second
}