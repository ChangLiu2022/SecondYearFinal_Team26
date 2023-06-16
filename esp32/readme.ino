#include <HardwareSerial.h>
#include <WiFi.h>
#include <time.h>
#include <WString.h>
#include <HTTPClient.h>
#include <bitset>
#include <stdint.h>
#define RXD2 16
#define TXD2 17
#include <WiFiClient.h>
#include <vector>

uint8_t receivedByte[4];

const int uartbaudrate = 115200;
HardwareSerial FPGA(2);
bool receivingCoordinates = false;
uint8_t beaconData [36];
uint8_t receivedData [4];
uint8_t coordsData [4];
uint8_t aaaa;

String BeaconType;
char package[256];



//WiFiServer WifiServer(12000);
//WiFiClient client = WifiServer.available();

//Server we are sending the data to and connection timeout in ms
//const char *ip = "3-8-159-120";
//const uint port = 12000;
//const int timeout = 3000;

//IP:3-8-159-120
//Port: 12000

//int transactionCount = 0;
int i = 0;
int j = 0;
int k = 0;
int l = 0;

bool broken_yellow = false, broken_blue = false, broken_white = false;
int acounter = 0;

//Time formatting using an NTP Server
const char* ntpServer = "0.uk.pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;
char currentTime[26] = {'\0'};
char PostData = 256;

/*const char* ssid = "Anast";
const char* password = "aaaaaaaa";*/
const char* ssid = "Zhou Qidong (2)";
const char* password = "Zhouqidong";


void initWiFi() {

  // Serial.println("[WiFi] | Checking the Current Connection");

  if(WL_CONNECTED != WiFi.status()){
    // Serial.println("--------------[WIFI COMM]----------");
    // Serial.println("[WiFi] | Disconnected From Wifi");
    WiFi.mode(WIFI_STA);
    // Serial.print("[WiFi] | SSID:");
    // Serial.print(ssid);
    // Serial.print(" PSWD:");
    // Serial.println(password);

    WiFi.begin(ssid, password);
    //WiFi.begin("Anast","aaaaaaaa");
    // Serial.println("[WiFi] | Starting Connection");
    Serial.println("haha");
    while (WiFi.status() != WL_CONNECTED) {
      // Serial.println("[WiFi] | Waiting for Connection");
      Serial.println("connecting");
      delay(5000);
    }
    Serial.println("connected");
    // Serial.print("[WiFi] | Connected at: ");
    // Serial.println(WiFi.localIP());

  }else {

    // Serial.println("[WiFi] | WiFi is Connected");

  }}

  
bool printLocalTime(){
  initWiFi();
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("[NTP] | Failed to obtain time");
    return false;
  }
  strftime(currentTime, 26, "%FT%TZ", &timeinfo);
  Serial.println(currentTime);
  // Serial.print("[NTP] | ");
  // Serial.println(currentTime);
  return true;
  }

int size =0;

void setup() {
  Serial.begin(uartbaudrate);
  Serial.println("????");
  FPGA.begin(uartbaudrate, SERIAL_8N1, RXD2, TXD2);
  // Serial.println("====================[SETUP COMM]=================");
  //Connect to Wifi
  initWiFi();

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("endconfigtime");
  printLocalTime(); 
  Serial.println("end print local time");
  delay(1000);
  Serial.println("start");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("END");

}

uint8_t reverseBitsArray(uint8_t *array){
  uint8_t t0 = 0;
  uint8_t t1 = 0;
  uint8_t t2 = 0;
  uint8_t t3 = 0;

  t0 = array[0];
  t1 = array[1];
  t2 = array[2];
  t3 = array[3];

  array[3] = t0;
  array[2] = t1;
  array[1] = t2;
  array[0] = t3;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void sendPostRequest(uint16_t vPostData[78]) { // NCOORDS
  HTTPClient http;
  const char* serverIP = "18.168.204.27";
  const int serverPort = 3001;
  // Specify the target URL
  String url = "http://" + String(serverIP) + ":" + String(serverPort) + "/api/data";
  char PostData[256];
  
  // Start the HTTP POST request
  http.begin(url);
  // Set the content type to JSON
  http.addHeader("Content-Type", "application/json");

  printLocalTime();

  sprintf(PostData, "[{\"timestamp\":\"%s\", \"BeaconType\":\"REDTL\", \"xcordinate\":%d, \"ycordinate\":%d}]", currentTime, vPostData[0] , vPostData[1]);
  int httpResponseCode = http.POST(PostData);
  http.end();

  http.begin(url);
  // Set the content type to JSON
  http.addHeader("Content-Type", "application/json");
  sprintf(PostData, "[{\"timestamp\":\"%s\", \"BeaconType\":\"REDBR\", \"xcordinate\":%d, \"ycordinate\":%d}]", currentTime,  vPostData[2], vPostData[3]);
  httpResponseCode = http.POST(PostData);
  http.end();
  http.begin(url);
  // Set the content type to JSON
  http.addHeader("Content-Type", "application/json");
  Serial.println(String(vPostData[5]));
  sprintf(PostData, "[{\"timestamp\":\"%s\", \"BeaconType\":\"YELLOWTL\", \"xcordinate\":%d, \"ycordinate\":%d}]", currentTime, vPostData[4], vPostData[5]);
  Serial.println(String(PostData));
  httpResponseCode = http.POST(PostData);
  http.end();
  http.begin(url);
  // Set the content type to JSON
  http.addHeader("Content-Type", "application/json");  
  sprintf(PostData, "[{\"timestamp\":\"%s\", \"BeaconType\":\"YELLOWBR\", \"xcordinate\":%d, \"ycordinate\":%d}]", currentTime, vPostData[6], vPostData[7]);
  httpResponseCode = http.POST(PostData);
  http.end();
  http.begin(url);
  // Set the content type to JSON
  http.addHeader("Content-Type", "application/json");
  sprintf(PostData, "[{\"timestamp\":\"%s\",  \"BeaconType\":\"BLUETL\", \"xcordinate\":%d, \"ycordinate\":%d}]", currentTime, vPostData[8], vPostData[9]);
  httpResponseCode = http.POST(PostData);
  http.end();
  http.begin(url);
  // Set the content type to JSON
  http.addHeader("Content-Type", "application/json");
  sprintf(PostData, "[{\"timestamp\":\"%s\",  \"BeaconType\":\"BLUEBR\", \"xcordinate\":%d, \"ycordinate\":%d}]", currentTime, vPostData[10], vPostData[11]);
  httpResponseCode = http.POST(PostData);
  http.end();

  for (int i = 12; i<45; i++){//NCOORDS
    http.begin(url);
  // Set the content type to JSON
    http.addHeader("Content-Type", "application/json");
    sprintf(PostData, "[{\"timestamp\":\"%s\", \"BeaconType\":\"WL\", \"xcordinate\":%d, \"ycordinate\":%d}]", currentTime,  vPostData[i], vPostData[i+1]);
    httpResponseCode = http.POST(PostData);
    http.end();
  }
  for (int i =0; i< 45; i++)//NCOORDS
    vPostData = 0;
  
  // Check the response code
  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  
  // End the HTTP connection

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  uint16_t array_package[78];//NCOORDS
  bool PackageLost = true;
  broken_yellow = false;
  broken_blue = false;
  broken_white = false;
  unsigned int xcordinate = 0;
  unsigned int ycordinate = 0;

  Serial.println("waiting for fpga");


  if (FPGA.available()) {
    size = 0;
    int state = 0;
    
    Serial.print("AWAITING 0RBB");
    while(state != 4){

      FPGA.readBytes(&aaaa, 1);
      Serial.print("Received: ");
      Serial.print(String(aaaa,BIN)); 
      Serial.print("   State: ");
      Serial.println(String(state)); 

      if(state == 0 && aaaa == 0x42){
        state = 1;
      }
      else if(state == 1 && aaaa == 0x42){
        state = 2;
      }
      else if(state == 2 && aaaa == 0x52){
        state = 3;
      }
      else if(state == 3 && aaaa == 0x00){
        Serial.println("CHANGED to 4 should jump out");
        state = 4;
      }
      else{
        state = 0;
      }
    }

    
    /*  for(i = 0; i <=3;i++){
        FPGA.readBytes(&aaaa, 1);
        Serial.println(String(i));
        Serial.println(String(aaaa,BIN)); 
        beaconData[i] = aaaa;
      }
    if (beaconData[3]==0x00){
      if (beaconData[2]==0x52){
        if (beaconData[1]==0x42){
          if (beaconData[0]==0x42){
            PackageLost = false;
          }
        }
      }
      else Serial.println("Package was lost");
    }
    */
    reverseBitsArray(beaconData);
    for(i = 0; i <=3;i++){
      FPGA.readBytes(&aaaa, 1);
      Serial.println(String(i));
      Serial.println(String(aaaa,BIN)); 
      beaconData[i] = aaaa;
    }
    reverseBitsArray(beaconData);
    xcordinate = (beaconData[0] <<8) | (beaconData[1]);


    array_package[size] = xcordinate;
    size ++;
    
    ycordinate = (beaconData[2]<< 8 ) | (beaconData[3]);
    Serial.println("!!!!!!!!!!!!!!!!!");
    Serial.print(String(beaconData[2]));
    Serial.println(String(beaconData[3]));
    Serial.println(String(ycordinate));
    Serial.println(String(size));
    Serial.println("!!!!!!!!!!!!!!!!!");
    array_package[size] =ycordinate;
    size ++;
    BeaconType = "REDTL";
    Serial.print("RED top left: ");
    Serial.print(beaconData[0]);
    Serial.print(beaconData[1]);
    Serial.print(beaconData[2]);
    Serial.println(beaconData[3]);



    for(i = 0; i <=3;i++){
      FPGA.readBytes(&aaaa, 1);
      Serial.println(String(i));
      Serial.println(String(aaaa,BIN)); 
      beaconData[i] = aaaa;
    }
    reverseBitsArray(beaconData);
    Serial.print("RED bottom right: ");
    xcordinate = (beaconData[0] << 8) | (beaconData[1]);
    array_package[size] = xcordinate;
    size ++;
    ycordinate = (beaconData[2] << 8 ) | (beaconData[3]);
    array_package[size] = ycordinate;
    size ++;
    BeaconType = "REDBR";
    Serial.print(beaconData[0]);
    Serial.print(beaconData[1]);
    Serial.print(beaconData[2]);
    Serial.println(beaconData[3]);
    Serial.print(xcordinate);
    Serial.println(ycordinate);

    Serial.print("should be yellow beacon");
    state = 0;
    while(state != 4){

      FPGA.readBytes(&aaaa, 1);
      Serial.print("Received: ");
      Serial.print(String(aaaa,BIN)); 
      Serial.print("   State: ");
      Serial.println(String(state)); 

      if(state == 0 && aaaa == 0x42){
        state = 1;
      }
      else if(state == 1 && aaaa == 0x42){
        state = 2;
      }
      else if(state == 2 && aaaa == 0x59){
        state = 3;
      }
      else if(state == 3 && aaaa == 0x00){
        Serial.println("CHANGED to 4 should jump out");
        state = 4;
      }
      else{
        state = 4;
        broken_yellow = true;
      }
    }
  for(i = 0; i <=3;i++){
      FPGA.readBytes(&aaaa, 1);
      Serial.println(String(i));
      Serial.println(String(aaaa,BIN)); 
      beaconData[i] = aaaa;
    }
    reverseBitsArray(beaconData);
    Serial.print("YEL top left: ");
    Serial.print(beaconData[0]);
    Serial.print(beaconData[1]);
    Serial.print(beaconData[2]);
    Serial.println(beaconData[3]);
    if(!broken_yellow){
      xcordinate = (beaconData[0] << 8) | (beaconData[1]);
      ycordinate = (beaconData[2] << 8 ) | (beaconData[3]);
    } else {
       xcordinate = ((beaconData[0] << 8) | (beaconData[1]))|0x8000;
       ycordinate = ((beaconData[2] << 8 ) | (beaconData[3]))|0x8000;
    }
    array_package[size] = xcordinate;
    Serial.println("============");
    Serial.println(String(size));
    Serial.println(String(xcordinate));
    Serial.println("============");
    size ++;

    Serial.println(String(ycordinate));
    Serial.println(String(beaconData[2]));
    array_package[5] = ycordinate;
    
    Serial.println("============");
    Serial.println(String(size));
    Serial.println(String(array_package[size]));
    Serial.println(String(array_package[5]));
    Serial.println(String(ycordinate));
    Serial.println("============");
    size ++;

    Serial.print(xcordinate);
    Serial.println(ycordinate);
    BeaconType = "YELLOWTL";
    
    
    for(i = 0; i <=3;i++){
      FPGA.readBytes(&aaaa, 1);
      Serial.println(String(i));
      Serial.println(String(aaaa,BIN)); 
      beaconData[i] = aaaa;
    }
    reverseBitsArray(beaconData);
    Serial.print("YEL bottom right: ");
    BeaconType = "YELLOWBR";
    Serial.print(beaconData[0]);
    Serial.print(beaconData[1]);
    Serial.print(beaconData[2]);
    Serial.println(beaconData[3]);
    xcordinate = (beaconData[0] << 8) | (beaconData[1]);
    array_package[size] = xcordinate;
    size ++;

    ycordinate = (beaconData[2] << 8 ) | (beaconData[3]);
    array_package[size] = ycordinate;
    size ++;

    Serial.print(xcordinate);
    Serial.println(ycordinate);

    Serial.println(String(array_package[5]));
    Serial.print("should be blue beacon");

    state = 0;

    while(state != 4){

      FPGA.readBytes(&aaaa, 1);
      Serial.print("Received: ");
      Serial.print(String(aaaa,BIN)); 
      Serial.print("   State: ");
      Serial.println(String(state)); 

      if(state == 0 && aaaa == 0x42){
        state = 1;
      }
      else if(state == 1 && aaaa == 0x42){
        state = 2;
      }
      else if(state == 2 && aaaa == 0x47){
        state = 3;
      }
      else if(state == 3 && aaaa == 0x00){
        Serial.println("CHANGED to 4 should jump out");
        state = 4;
      }
      else{
        state = 4;
        broken_blue = true;
      }
    }
    for(i = 0; i <=3;i++){
      FPGA.readBytes(&aaaa, 1);
      Serial.println(String(i));
      Serial.println(String(aaaa,BIN)); 
      beaconData[i] = aaaa;
    }
    reverseBitsArray(beaconData);
    Serial.print("BLU top left: ");
    xcordinate = (beaconData[0] << 8) | (beaconData[1]);
    array_package[size] = xcordinate;
    size ++;
    ycordinate = (beaconData[2] << 8 ) | (beaconData[3]);
    array_package[size] = ycordinate;
    size ++;
    BeaconType = "BLUETL";
    Serial.print(beaconData[0]);
    Serial.print(beaconData[1]);
    Serial.print(beaconData[2]);
    Serial.println(beaconData[3]);
    Serial.print(xcordinate);
    Serial.println(ycordinate);
    
    for(i = 0; i <=3;i++){
      FPGA.readBytes(&aaaa, 1);
      Serial.println(String(i));
      Serial.println(String(aaaa,BIN)); 
      beaconData[i] = aaaa;
    }
    reverseBitsArray(beaconData);
    Serial.print("BLU bottom right: ");
    Serial.print(beaconData[0]);
    Serial.print(beaconData[1]);
    Serial.print(beaconData[2]);
    Serial.println(beaconData[3]);
    if(!broken_blue){
      xcordinate = (beaconData[0] << 8) | (beaconData[1]);
      ycordinate = (beaconData[2] << 8 ) | (beaconData[3]);
    } else {
       xcordinate = ((beaconData[0] << 8) | (beaconData[1]))|0x8000;
       ycordinate = ((beaconData[2] << 8 ) | (beaconData[3]))|0x8000;
    }
    //xcordinate = (beaconData[0] << 8) | (beaconData[1]);
    array_package[size] = xcordinate;
    size ++;
    //ycordinate = (beaconData[2] << 8 ) | (beaconData[3]);
    array_package[size] = ycordinate;
    size ++;
    Serial.print(xcordinate);
    Serial.println(ycordinate);
    BeaconType = "BLUEBR";
    Serial.println(BeaconType);
    state = 0;
    while(state != 4){

      FPGA.readBytes(&aaaa, 1);
      Serial.print("Received: ");
      Serial.print(String(aaaa,BIN)); 
      Serial.print("   State: ");
      Serial.println(String(state)); 

      if(state == 0 && aaaa == 0x42){
        state = 1;
      }
      else if(state == 1 && aaaa == 0x42){
        state = 2;
      }
      else if(state == 2 && aaaa == 0x57){
        state = 3;
      }
      else if(state == 3 && aaaa == 0x00){
        Serial.println("CHANGED to 4 should jump out");
        state = 4;
      }
      else{
        state = 4;
        broken_white = true;
      }
    }
    if(!broken_white){
      Serial.println("Coords: ");
      bool isEnd = false;
      int counter = 0;
      while(!isEnd && counter <= 33){//TODO: do shit here to detect the ending of coordinate sendin
        for(k= 0; k<=3;k++){
          FPGA.readBytes(&aaaa, 1);
          Serial.println("coords x and y:");
          Serial.println(String(aaaa,BIN));
          coordsData[k] = aaaa;
        }

        reverseBitsArray(coordsData);
        Serial.println("?????????");
        Serial.println(String(coordsData[0], BIN));
        Serial.println(String(coordsData[1], BIN));
        Serial.println(String(coordsData[2], BIN));
        Serial.println(String(coordsData[3], BIN));
        
        isEnd = coordsData[0] == 0x00 && coordsData[1] == 0x45 && coordsData[2] == 0x4e && coordsData[3] == 0x44;


        Serial.print("isEND:");
        Serial.println(String(isEnd));  
        Serial.println(String(counter));
        Serial.println("?????????");
        if(!broken_white){
            xcordinate = (coordsData[0] << 8) | (coordsData[1]);
            ycordinate = (coordsData[2] << 8 ) | (coordsData[3]);
        } else {
            xcordinate = ((coordsData[0] << 8) | (coordsData[1]))|0x8000;
            ycordinate = ((coordsData[2] << 8 ) | (coordsData[3]))|0x8000;
            Serial.println("WHITE IS BROKEN");
            Serial.println(broken_white);
         }
      
        if (!isEnd){
            array_package[size] = xcordinate;
            size ++;
            array_package[size] = ycordinate;
            size ++;
        }

        BeaconType = "WL";
        Serial.print("coords x and y");
        Serial.print(coordsData[0]);
        Serial.print(coordsData[1]);
        Serial.print(coordsData[2]);
        Serial.println(coordsData[3]);
        Serial.print(xcordinate);
        Serial.println(ycordinate);
        counter++;
      }
    }
  Serial.println("send");
  Serial.println(String(array_package[5]));
  sendPostRequest(array_package);
  
  Serial.println("===========SUMMARY=========");
  Serial.print("Yellow:");
  Serial.print(String(broken_yellow));
  Serial.print(" Blue:");
  Serial.print(String(broken_blue));
  Serial.print(" White:");
  Serial.print(String(broken_white));
  Serial.print(" Counter:");

  

  if(broken_yellow || broken_blue || broken_white){
    acounter++;
  }
  Serial.println(String(acounter));
  Serial.println("==========ENDSUMMARY============");

//E
  }

}
