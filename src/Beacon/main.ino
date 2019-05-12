/*
 * This script is written for the ESP32 chip series.
 * The core funciton of the script is to let the chip set behave as a client node, in a server-clint model.
 * 
 * The client is equipped with a HMC5883 Magnetometer digital compass. The node sends the information from itself to
 * the server.
 * The server then computes the vector based on the distance and the compass reading. The bearing and the heading is used to calculate the
 * the heading in degrees.
 * 
 */
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include "HMC5883L.h"

#define REFRESH_INTERVAL 5000
#define BAUD_RATE        115200
#define SENSOR_ID        102
#define LED              16
#define BTN              27
// configure the compass as reqiured
#define OSR 0b00               // over sampling rate set to 512. 0b01 is 256, 0b10 is 128 and 0b11 is 64
#define RNG 0b00               // Full Scale set to +/- 2 Gauss, 0b01 is +/- 8.
#define ODR 0b00               // output data rate set to 10Hz, 0b01 is 50Hz, 0b10 is 100Hz, 0b11 is 200Hz
#define MODE 0b01              // continuous measurement mode, 0b00 is standby
#define CR2 0b00000000          // control register 2: disable soft reset and pointer rollover, interrupt pin not enabled
#define RESETPERIOD 0b00000001  // Datasheet recommendation.

const char* ssid     = "API2";
const char* password = "lawl123456";
const char* host     = "10.0.0.1";
int         port     = 4242;

QMC5883L    compass;          // Compass object.
int         error    = 0;
long long   timer    = 0;
WiFiMulti   wifiMulti;        // Multi connection WiFi node instance.

void setup() {
    Serial.begin(BAUD_RATE);
    delay(500);
    pinMode(LED, OUTPUT);
    pinMode(BTN, INPUT);    
    digitalWrite(BTN, LOW);
    Wire.begin();
    compass = QMC5883L();   
    // This snippet of codes let aserts the connectivity of the compass module.
    do {
      delay(100);
      Wire.beginTransmission(QMC5883L_Address);
      error = Wire.endTransmission();
      if (error) Serial.println("Compass failure! Check wiring!");
    }while(error);

    // Setting up basic registers of the compass module.
    compass.dataRegister.OSR_RNG_ODR_MODE = (OSR << 6) |(RNG << 4)  | (ODR <<2) |  MODE;
    compass.dataRegister.CR2_INT_ENABLE = CR2;
    compass.dataRegister.SET_RESET_PERIOD = RESETPERIOD;

    Serial.println("Configuring QMC5883L | OSR 512, range +/-2 Gauess, ODR 10, Continous");

    // Compass configuration of registers for setting it in continous mode.
    error = compass.Configure(compass.dataRegister);
    if(error != 0)
      Serial.println(compass.GetErrorText(error));
}


int getStrength(int points){
  // A function for returning the signal strength with an averaged value.
  
    long rssi = 0;
    long averageRSSI=0;
    for (int i=0;i < points;i++){
        rssi += WiFi.RSSI();
        delay(5);
    }
    averageRSSI=rssi/points;
    return averageRSSI;
}

float bearingDegrees(float headingDegrees) {  
  // A function returns the bearing angles.   
   float bearing = headingDegrees - 90;
   if (bearing < 0) {
      bearing += 360;
     }
   return bearing;
}

void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
{
  
  Serial.println("Bearing: " + String(bearingDegrees(headingDegrees)) + " Heading: " + String(headingDegrees));
  int bearing = bearingDegrees(headingDegrees);
  if((bearing > 337.5) || (bearing < 22.5))    Serial.print("North");
  if((bearing > 22.5)  && (bearing < 67.5 ))   Serial.print("North-East");
  if((bearing > 67.5)  && (bearing < 112.5 ))  Serial.print("East");
  if((bearing > 112.5) && (bearing < 157.5 ))  Serial.print("South-East");
  if((bearing > 157.5) && (bearing < 202.5 ))  Serial.print("South");
  if((bearing > 202.5) && (bearing < 247.5 ))  Serial.print("South-West");
  if((bearing > 247.5) && (bearing < 292.5 ))  Serial.print("West");
  if((bearing > 292.5) && (bearing < 337.5 ))  Serial.print("North-West");
}

float returnAngle() {
  // A function for calculating the return angles of the compass. 
  // The function uses two modes raw and scaled for calculating the heading degrees of magnetic true north.
  
  MagnetometerRaw raw = compass.ReadRawAxis(&compass.dataRegister);
  MagnetometerScaled scaled = compass.ReadScaledAxis(&compass.dataRegister);
  int Gauss_OnThe_XAxis = raw.ZAxis;
  float heading = atan2(scaled.YAxis, scaled.XAxis);

  float declinationAngle = 0.0457;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  float headingDegrees = heading * 180/M_PI;
  return headingDegrees;
}

void scanNetwork() {
  // A function for scanning the network for the best suitable network to connect too.
  
  WiFi.disconnect();
  wifiMulti.addAP("API1", "lawl123456");
  wifiMulti.addAP("API2", "lawl123456");
  while(wifiMulti.run() != WL_CONNECTED) {
    digitalWrite(LED, 0);
    delay(100);
    digitalWrite(LED, 1);
    delay(100);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to: " + WiFi.SSID());
}

void loop() {
  int disconnectCounter = 0;
  while(1) {
      if ((long long)millis() - (long long)timer >= 5000) {
       timer = millis();
       if((WiFi.status() != WL_CONNECTED)) {
        scanNetwork();
       }
       
       if((WiFi.status() == WL_CONNECTED)) {
        WiFiClient txClient;
        if(!txClient.connect(host, port)){
            Serial.println("Connection error");
        }
        else {
          StaticJsonDocument<128> doc;
          doc["distance"]    = getStrength(20);
          doc["orientation"] = returnAngle();
          doc["sensor_id"]   = SENSOR_ID;
          String payload;
          serializeJson(doc, payload);
          Serial.println("Distance: " + String(getStrength(20)) + " orientation: " + String(returnAngle()));
          txClient.println(payload);
          digitalWrite(LED, HIGH);
          delay(100);
          digitalWrite(LED, LOW);
          delay(100);
      }
    }
    else {
      disconnectCounter++;
      if (disconnectCounter >= 5) {
        Serial.println("Restarting chip");
        ESP.restart();
      }
      Serial.println("Restarting in: " + String(5 - disconnectCounter));
    }
   }
  }
}
