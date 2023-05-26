#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <K30_I2C.h>
#include <SensirionI2CSen5x.h>
#include <WiFi.h>
#include <esp_now.h>

#include <TinyGPS++.h>


#include <time.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#define SS 34
#define MOSI 35
#define MISO 37
#define SCK 36
//SPIClass spi = SPIClass(VSPI);
 // SPI.begin(SCK, MISO, MOSI, SS);

#define RXD2 16
#define TXD2 17
RTC_DATA_ATTR int readingID = 0;
int last_second, Second, Minute, Hour, Day, Month;
int Year;
const int UTC_offset = 2;
void initSDCard(){
  while (!SD.begin(SS,SPI,80000000)) {
    Serial.println("Card Mount Failed");
    delay(200);
    //return;
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
}
uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x46, 0xFC, 0xEC};
//uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x46, 0xFE, 0x12};
String success;
typedef struct struct_message {
/*     int readingId2;
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
    float noxIndex_;   */ 
    String message;
} struct_message;
//How to send data
struct_message sendingdata;
struct_message receivedata;
esp_now_peer_info_t peerInfo;

String dataMessage;
// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.println(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}
void logSDCard(    float massConcentrationPm1p0,float massConcentrationPm2p5,float massConcentrationPm4p0,float massConcentrationPm10p0,float ambientHumidity,float ambientTemperature,float vocIndex,float noxIndex, float co2 ) {
  dataMessage = String(readingID) + "," + String(Year) + "/" + String(Month) + "/" + String(Day) + "-" + String(Hour) + "-" + String(Minute) + "-" + String(Second) + "," + String(readingID) + "," + String(massConcentrationPm1p0) + "," + String(massConcentrationPm2p5) + "," + String(massConcentrationPm4p0) + "," + String(massConcentrationPm10p0) + "," + String(ambientHumidity) + "," + String(ambientTemperature) + "," + String(vocIndex)+ "," + String(noxIndex) + "," + 
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
  readingID++;
}
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.println(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
  
}


//-33.932743790851305, 18.85852673919876
// Define the reference point coordinates
float ref_lat = -33.932743790851305;
float ref_lon = 18.85852673919876;

// Define the radius in meters
float radius = 100;
// variable definitions
// Define a flag to indicate if the function has been performed
bool flag = false;

// Define your function
void sendToESP() {
File file = SD.open("/data.csv", FILE_READ);
  if(!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  while (file.available())
  {
    String line = file.readStringUntil('\n');
    sendingdata.message = line;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendingdata, sizeof(sendingdata));

  }
  

  file.close();



  // Do something
}

long double toRadians(const long double degree) 
{ 
    long double one_deg = (M_PI) / 180; 
    return (one_deg * degree); 
} 
// Define a function to calculate the distance between two points using Haversine formula
float distance(float lat1, float lon1, float lat2, float lon2) {
  // Use the formula from https://stackoverflow.com/questions/63916877/gps-distance-calculations-between-two-points-using-arduino

      // Convert the latitudes and longitudes from degree to radians. 
    lat1 = toRadians(lat1); 
    lon1 = toRadians(lon1); 
    lat2 = toRadians(lat2); 
    lon2 = toRadians(lon2); 
      
    // Haversine Formula 
    long double dlong = lon2 - lon1; 
    long double dlat = lat2 - lat1; 
  
    long double ans = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlong / 2), 2);
    ans = 2 * asin(sqrt(ans)); 
  
    // Radius of Earth in Kilometers, R = 6371 
    // Use R = 3956 for miles 
    long double R = 6371; 
      
    // Calculate the result 
    ans = ans * R; 
  
    return ans; 
}




time_t prevDisplay = 0; // Count for when time last displayed
// Define the serial connection to the A7 module
#define GPS Serial1
//SoftwareSerial gpsSerial(RX_PIN, TX_PIN);
TinyGPSPlus gps;

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
//uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x46, 0xFD, 0x4A};
//uint8_t localCustomMac [] = {};



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
   //Serial.println(receivedata.ambientTemperature_);

 }



//end

void setup() {
  //SPI.begin(SCK,)
  SPI.begin(SCK, MISO, MOSI, SS);
  //spi.begin();
  // put your setup code here, to run once:
  Serial.begin(115200);
    delay(3000);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
    delay(3000);
  Serial.println("Start up");  
  Serial.println(TinyGPSPlus::libraryVersion());

  initSDCard();
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
  WiFi.mode(WIFI_STA);
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



//sendingdata.co2_ = co2;
//esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendingdata, sizeof(sendingdata));
int period = 10000;
unsigned long time_now = 0;
double lastlat;
double lastlon;


void loop() {
  // put your main code here, to run repeatedly:
  if(millis() >= time_now + period){
        time_now += period;
  //k30
  int co2; // Variable to store CO2 value
  int status; // Variable to store status of reading
  status = k30.readCO2(co2); // Read CO2 value from sensor
  if (status == 0) { // If reading was successful
    //Serial.print("CO2:"); // Print CO2 label
    //Serial.print(co2); // Print CO2 value
    //Serial.print(" ppm"); // Print ppm unit
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
        Serial.print( "\t");
        Serial.print("NoxIndex:");
        if (isnan(noxIndex)) {
            Serial.println("n/a");
        } else {
            Serial.println(noxIndex);
        } */
    } 

/*     sendingdata.co2_ = co2;
    sendingdata.massConcentrationPm1p0_ = massConcentrationPm1p0;
    sendingdata.massConcentrationPm2p5_ = massConcentrationPm2p5;
    sendingdata.massConcentrationPm4p0_ = massConcentrationPm4p0;
    sendingdata.massConcentrationPm10p0_ = massConcentrationPm10p0;
    sendingdata.ambientHumidity_ = ambientHumidity;
    sendingdata.ambientTemperature_ = ambientTemperature;
    sendingdata.vocIndex_ = vocIndex;
    sendingdata.noxIndex_ = noxIndex;  
    sendingdata.lat_ = lastlat;
    sendingdata.lon_ = lastlon;
    sendingdata.Second = Second;
    sendingdata.Minute = Minute;
    sendingdata.Hour = Hour;
    sendingdata.Day = Day;
    sendingdata.Month = Month;
    sendingdata.Year = Year;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendingdata, sizeof(sendingdata)); */
  logSDCard(massConcentrationPm1p0,massConcentrationPm2p5,massConcentrationPm4p0,massConcentrationPm10p0,ambientHumidity,ambientTemperature,vocIndex,noxIndex,co2);
  time_now = millis();

  }
  //delay(10000); // Wait for 10 seconds
    while (GPS.available()) {
    // Read a byte of data from the GPS module
    char c = GPS.read();

    // Write the byte to the serial monitor
   // Serial.write(c);

    // Feed the byte to the TinyGPSPlus object
    gps.encode(c);

    // Check if a new location is available
    if (gps.location.isUpdated()) {
      lastlat = gps.location.lat();
      lastlon = gps.location.lng();
      Serial.println("");
      Serial.println("");
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
      if (gps.time.isValid())
      {
        Minute = gps.time.minute();
        Second = gps.time.second();
        Hour   = gps.time.hour();
      }    
            // get date drom GPS module
      if (gps.date.isValid())
      {
        Day   = gps.date.day();
        Month = gps.date.month();
        Year  = gps.date.year();
      }
      if(last_second != gps.time.second())  // if time has changed
      {
        last_second = gps.time.second();
        Serial.print("time: ");
        Serial.print(Year);
        Serial.print("/");
        Serial.print(Month);
        Serial.print("/");
        Serial.println(Day); 
        Serial.print("-");
        Serial.print(Hour);
        Serial.print(":");
        Serial.print(Minute);
        Serial.print(":");
        Serial.println(Second); 

      }
      float dist = distance(ref_lat, ref_lon, lastlat, lastlon);
      if (dist <= radius) {
        // Check if the function has been performed
        if (!flag) {
          // Perform the function
          sendToESP();
          // Set the flag to true
          flag = true;
        }
      } 
      else {
      // Reset the flag to false
       flag = false;
      }     
    
    }

  }
}