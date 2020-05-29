
#include <SPI.h>
#include <LoRa.h>
/////////////////////////////////////////////WIFI FIREBASE////////////////////////////////////////////////////
#include <WiFi.h>                                                // esp32 library
#include <IOXhop_FirebaseESP32.h>                                // firebase library
    
/////////////////////////////////////////////WIFI FIREBASE////////////////////////////////////////////////////

//define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2


/////////////////////////////////////////////WIFI FIREBASE///////////////////////////////////////////////////
#define FIREBASE_HOST "colatmos.firebaseio.com"//"colpm-82125.firebaseio.com"                         // the project name address from firebase id
#define FIREBASE_AUTH "0qZ2IaeygOzxB2MJFCQ3lRpc08IqUpQUEnftzw9I"                    // the secret key generated from firebase
#define WIFI_SSID "BIG_M" //เปลี่ยนนนนนนนนน                                         // input your home or public wifi name
#define WIFI_PASSWORD "XES3FR7Q" //เปลี่ยนนนนนนนนน

String Counter,Humid,Temp,P25,P10,Year,Month,Day,Hour,Minit,Wind,Uv;
String fireStatus = "";  
/////////////////////////////////////////////WIFI FIREBASE///////////////////////////////////////////

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");
  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
  delay(500); 
/////////////////////////////////////////////WIFI FIREBASE///////////////////////////////////////////
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);                                      //try to connect with wifi
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP Address is : ");
  Serial.println(WiFi.localIP());                                                      //print local IP address
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);                                       // connect to firebase
 
  Firebase.setString("Counter", "0");
  Firebase.setString("Humid", "0");
  Firebase.setString("Temp", "0");
  Firebase.setString("P25", "0");
  Firebase.setString("P10", "0");
  Firebase.setString("Year", "0");
  Firebase.setString("Month", "0");//send initial string of led status
  Firebase.setString("Day", "0");
  Firebase.setString("Hour", "0");
  Firebase.setString("Minit", "0");
  Firebase.setString("Wind", "0");
  Firebase.setString("Uv", "0"); 
 /////////////////////////////////////////////WIFI FIREBASE///////////////////////////////////////////
  
}

void loop() {
  // try to parse packet
  int packetSize  = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.println(LoRaData); 
      
      ////////////////////COUNTER COUNTER COUNTER COUNTER COUNTER//////////////////
      int countS = LoRaData.indexOf("!");
      int countL = LoRaData.lastIndexOf("/");
         Counter = LoRaData.substring((countS+1),countL-countS);
      ////////////////////COUNTER COUNTER COUNTER COUNTER COUNTER//////////////////
         
      ////////////////////HUMID HUMID HUMID HUMID HUMID//////////////////
      int humidS = LoRaData.indexOf("/");
      int humidL = LoRaData.lastIndexOf("&");
          Humid = LoRaData.substring((humidS+1),humidL);
          if(Humid=="nan")Humid = "0";
      ////////////////////HUMID HUMID HUMID HUMID HUMID//////////////////
      
      ////////////////////TEMP TEMP TEMP TEMP TEMP//////////////////
      int tempS = LoRaData.indexOf("&");
      int tempL = LoRaData.lastIndexOf("#");
          Temp = LoRaData.substring((tempS+1),tempL);
          if(Temp== "nan")Temp = "0";
      ////////////////////TEMP TEMP TEMP TEMP TEMP//////////////////
      
      ////////////////////PM25 PM25 PM25 PM25 PM25//////////////////
      int p25S = LoRaData.indexOf("#");
      int p25L = LoRaData.lastIndexOf("*");
          P25 = LoRaData.substring((p25S+1),p25L);   
      ////////////////////PM25 PM25 PM25 PM25 PM25//////////////////
//      
      ////////////////////PM10 PM10 PM10 PM10 PM10//////////////////
      int p10S = LoRaData.indexOf("*");
      int p10L = LoRaData.lastIndexOf("(");
         P10 = LoRaData.substring((p10S+1),p10L);
      ////////////////////PM10 PM10 PM10 PM10 PM10//////////////////
//
      ////////////////////YEAR YEAR YEAR YEAR YEAR//////////////////
      int yearS = LoRaData.indexOf("(");
      int yearL = LoRaData.lastIndexOf(")");
         Year = LoRaData.substring((yearS+1),yearL);
      ////////////////////YEAR YEAR YEAR YEAR YEAR//////////////////
      
      ////////////////////MONTH MONTH MONTH MONTH MONTH//////////////////
      int monthS = LoRaData.indexOf(")");
      int monthL = LoRaData.lastIndexOf("@");
         Month = LoRaData.substring((monthS+1),monthL);
      ////////////////////MONTH MONTH MONTH MONTH MONTH//////////////////
      
      ////////////////////////DAY DAY DAY DAY DAY////////////////////////
      int dayS = LoRaData.indexOf("@");
      int dayL = LoRaData.lastIndexOf("-");
         Day = LoRaData.substring((dayS+1),dayL);
      ////////////////////////DAY DAY DAY DAY DAY////////////////////////

      ////////////////////////HOUR HOUR HOUR HOUR HOUR////////////////////////
      int hourS = LoRaData.indexOf("-");
      int hourL = LoRaData.lastIndexOf("$");
         Hour = LoRaData.substring((hourS+1),hourL);
      ////////////////////////HOUR HOUR HOUR HOUR HOUR////////////////////////
      
      ////////////////////////MINUTIUES MINUTIUES MINUTIUES////////////////////////
      int minitS = LoRaData.indexOf("$");
      int minitL = LoRaData.lastIndexOf("%");
         Minit = LoRaData.substring((minitS+1),minitL);
      ////////////////////////MINUTIUES MINUTIUES MINUTIUES////////////////////////     

      ////////////////////////WIND WIND WIND WIND WIND WIND ////////////////////////
      int windS = LoRaData.indexOf("%");
      int windL = LoRaData.lastIndexOf("+");
         Wind = LoRaData.substring((windS+1),windL);
      ////////////////////////WIND WIND WIND WIND WIND WIND////////////////////////
      
      ////////////////////////UVUVUVUVUVUVUVUVUVUVUVUVUVUVUV////////////////////////
      int uvS = LoRaData.indexOf("+");
      int uvL = LoRaData.lastIndexOf(">");
         Uv = LoRaData.substring((uvS+1),uvL);
      
      ////////////////////////UVUVUVUVUVUVUVUVUVUVUVUVUVUVUV////////////////////////
//      Serial.print(Counter);
//      Serial.print(" ");
//      Serial.print(Humid);
//      Serial.print(" ");
//      Serial.print(Temp);
//      Serial.print(" ");
//      Serial.print(P25);
//      Serial.print(" ");
//      Serial.print(P10);
//      Serial.print(" ");
//      Serial.print(Year);
//      Serial.print(" ");
//      Serial.print(Month);
//      Serial.print(" ");
//      Serial.print(Day);
//      Serial.print(" ");
//      Serial.print(Hour);
//      Serial.print(" ");
//      Serial.println(Minit);
      
      Firebase.setString("Counter", Counter);
      Firebase.setString("Humid", Humid);
      Firebase.setString("Temp", Temp);
      Firebase.setString("P25", P25);
      Firebase.setString("P10", P10);
      Firebase.setString("Year", Year);
      Firebase.setString("Month", Month);
      Firebase.setString("Day", Day);
      Firebase.setString("Hour", Hour);
      Firebase.setString("Minit", Minit);
      Firebase.setString("Wind", Wind);
      Firebase.setString("Uv", Uv);
    }
  }else{
    Serial.println("lostdata from: ");  
  }
  //delay(3000);
  }
