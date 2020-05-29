/*********
  Modified from the examples of the Arduino LoRa library
  More resources: https://randomnerdtutorials.com
*********/
#include "DHT.h"
#include "RTClib.h"
#include <SDS011.h>

#define DHTTYPE DHT22
#define DHTPIN 4
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

//define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2

RTC_DS3231 rtc;
 int stringLength = 0;  
int voltagel=0;
int UVIndex = 0;
int sensorUV = 0;

const int sensorPin = 25; //Defines the pin that the anemometer output is connected to
int sensorValue = 0; //Variable stores the value direct from the analog pin
float sensorVoltage = 0; //Variable that stores the voltage (in Volts) from the anemometer being sent to the analog pin
float windSpeed = 0; // Wind speed in meters per second (m/s)

float voltageConversionConstant = .004882814; //This constant maps the value provided from the analog read function,
//which ranges from 0 to 1023, to actual voltage, which ranges from 0V to 5V
int sensorDelay = 1000; //Delay between sensor readings, measured in milliseconds (ms)

//Anemometer Technical Variables
//The following variables correspond to the anemometer sold by Adafruit, but could be modified to fit other anemometers.
double x=0;
double y=0;
double a=0;
double b=0;

float voltageMin = .4; // Mininum output voltage from anemometer in mV.
float windSpeedMin = 0; // Wind speed in meters/sec corresponding to minimum voltage

float voltageMax = 2.0; // Maximum output voltage from anemometer in mV.
float windSpeedMax = 32;
int preWindSpeed=0;
float p10, p25;
int err;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
DHT dht(DHTPIN, DHTTYPE);
int counter = 0;
String LoRaMessage = "";
SDS011 my_sds;

#ifdef ESP32
HardwareSerial port(2);
#endif

void setup() {
  //initialize Serial Monitor
  dht.begin();
  my_sds.begin(&port);
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // If the RTC have lost power it will sets the RTC to the date & time this sketch was compiled in the following line
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

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

}


void loop() {

  //// get PM.25 PM.10  /////
  err = my_sds.read(&p25, &p10);
  //// get PM.25 PM.10  /////

  /////  get humid temp /////
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  /////  get humid temp /////

  ///// output temp humid ////
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");

  } else {

    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" *C ");
  }

  //// output temp humid ////

  DateTime now = rtc.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();


  ///// OUTPUT PM //////
  if (!err) {
    Serial.println("P2.5: " + String(p25));
    Serial.println("P10:  " + String(p10));
  }
  ///// OUTPUT PM //////

  ///// output temp humid ////
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");

  } else {

    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" *C ");
  }
  //// output temp humid ////
  //// output Anemometer/////

  //// output Anemometer/////
  sensorValue = analogRead(sensorPin); //Get a value between 0 and 1023 from the analog pin connected to the anemometer

  sensorVoltage = sensorValue * voltageConversionConstant; //Convert sensor value to actual voltage

  //Convert voltage value to wind speed using range of max and min voltages and wind speed for the anemometer
  if (sensorVoltage <= voltageMin) {
    windSpeed = 0; //Check if voltage is below minimum value. If so, set wind speed to zero.
  }

  else {
    windSpeed = (sensorVoltage - voltageMin) * windSpeedMax / (voltageMax - voltageMin); //*2.232694 For voltages above minimum value, use the linear relationship to calculate wind speed.
  }
  x = windSpeed;
  if(x >= y){
    y=x;
  }else{
    y=y;
  }

  a = sensorVoltage;
  if (a >= b) {
    b=a;
  }else{
    b=b;
  }
  if (windSpeed != preWindSpeed){
    preWindSpeed=windSpeed;
  }
  
  //// output Anemometer/////


  //// UV UVU UUVUVUVUVUVUVUV//////
sensorUV = analogRead(35);                        //connect UV sensor to Analog 0   
   voltagel = (sensorUV * (5.0 / 1023.0))*1000;  //voltagel in miliVolts
  
  if(voltagel<50)
  {
    UVIndex = 0;
  }else if (voltagel>50 && voltagel<=227)
  {
    UVIndex = 0;
  }else if (voltagel>227 && voltagel<=318)
  {
    UVIndex = 1;
  }
  else if (voltagel>318 && voltagel<=408)
  {
    UVIndex = 2;
  }else if (voltagel>408 && voltagel<=503)
  {
    UVIndex = 3;
  }
  else if (voltagel>503 && voltagel<=606)
  {
    UVIndex = 4;
  }else if (voltagel>606 && voltagel<=696)
  {
    UVIndex = 5;
  }else if (voltagel>696 && voltagel<=795)
  {
    UVIndex = 6;
  }else if (voltagel>795 && voltagel<=881)
  {
    UVIndex = 7;
  }
  else if (voltagel>881 && voltagel<=976)
  {
    UVIndex = 8;
  }
  else if (voltagel>976 && voltagel<=1079)
  {
    UVIndex = 9;
  }
  else if (voltagel>1079 && voltagel<=1170)
  {
    UVIndex = 10;
  }else if (voltagel>1170)
  {
    UVIndex = 11;
  }


  //////UVUVUVUVUVUV////////////
  delay(5000);  
  ///// send radio /////

  Serial.print("Sending packet: ");
  Serial.println(counter);

  LoRaMessage = "!" + String(counter) + "/" + String(h) + "&" + String(t) + "#" + String(p25) + "*" + String(p10) + "(" + String(now.year(), DEC) + ")" + String(now.month(), DEC)
                + "@" + String(now.day(), DEC) + "-" + String(now.hour(), DEC) + "$" + String(now.minute(), DEC) + "%" + String(windSpeed) + "+" + String(UVIndex) + ">";
  Serial.println(LoRaMessage);

  LoRa.beginPacket();
  LoRa.print(LoRaMessage);
  LoRa.endPacket();

  //Send LoRa packet to receiver
  //  LoRa.beginPacket();
  //  LoRa.print("D ");
  //  LoRa.print(counter);
  //  LoRa.endPacket();
  //
  //  LoRa.beginPacket();
  //  LoRa.print("T ");
  //  LoRa.print(h); // send humid
  //  LoRa.print(t); // send temp
  //  LoRa.endPacket();
  //
  //  LoRa.beginPacket();
  //  LoRa.print("P");
  //  LoRa.print(p25); //send pm25
  //  LoRa.print(p10); //send pm10
  //  LoRa.endPacket();
  //  // send dats time //
  //  LoRa.print(now.year(), DEC);
  //  LoRa.print(now.month(), DEC);
  //  LoRa.print(now.day(), DEC);
  //  LoRa.print(daysOfTheWeek[now.dayOfTheWeek()]);
  //  LoRa.print(now.hour(), DEC);
  //  LoRa.print(now.minute(), DEC);
  //  LoRa.print(now.second(), DEC);
  //  // send dats time //
  //LoRa.endPacket();

  counter++;

  ///// send radio /////

}
