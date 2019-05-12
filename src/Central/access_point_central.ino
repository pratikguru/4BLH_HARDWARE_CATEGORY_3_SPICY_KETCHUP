/*
 * This script is written for the ESP32 chip series.
 * The core funciton of the script is to let the chip set behave as a client node, in a server-clint model.
 * 
 * The client is equipped with a HMC5883 Magnetometer digital compass. The node sends the information from itself to
 * the server.
 * The server then computes the vector based on the distance and the compass reading. The bearing and the heading is used to calculate the
 * the heading in degrees.
 * 
 * This script behaves as the central node that connects to another gateway.
 */
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include "HMC5883L.h";
#include <WiFiClient.h>


#define POST_REQUEST 1
#define GET_REQUEST  2

#define ROOM_ID      2          // Room ID has to be changed.
#define OSR 0b00               // over sampling rate set to 512. 0b01 is 256, 0b10 is 128 and 0b11 is 64
#define RNG 0b00               // Full Scale set to +/- 2 Gauss, 0b01 is +/- 8.
#define ODR 0b00               // output data rate set to 10Hz, 0b01 is 50Hz, 0b10 is 100Hz, 0b11 is 200Hz
#define MODE 0b01              // continuous measurement mode, 0b00 is standby
#define CR2 0b00000000          // control register 2: disable soft reset and pointer rollover, interrupt pin not enabled
#define RESETPERIOD 0b00000001  // Datasheet recommendation.

const char*  ssid;
const char*  password;
WiFiServer   server(4242);
QMC5883L     compass;
int          error    = 0;
WiFiMulti    WiFiMulti;

void setWiFi(const char* ssid, const char* password);
void connectWifi(const char* ssid, const char* password);

IPAddress local_IP(10, 0, 0, 1);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);
long long timer;

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n");
  setWiFi("API2", "lawl123456");
  Wire.begin();
  compass = QMC5883L();
  do {
      delay(100);
      Wire.beginTransmission(QMC5883L_Address);
      error = Wire.endTransmission();
      if (error) Serial.println("Compass failure! Check wiring!");
    }while(error);
    compass.dataRegister.OSR_RNG_ODR_MODE = (OSR << 6) |(RNG << 4)  | (ODR <<2) |  MODE;
    compass.dataRegister.CR2_INT_ENABLE = CR2;
    compass.dataRegister.SET_RESET_PERIOD = RESETPERIOD;

    Serial.println("Configuring QMC5883L | OSR 512, range +/-2 Gauess, ODR 10, Continous");
    error = compass.Configure(compass.dataRegister);
    if(error != 0)
      Serial.println(compass.GetErrorText(error));
}

float bearingDegrees(float headingDegrees) {     
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

void connectWifi(const char* ssid, const char* password, int requestType, String url, String payload) {
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  
  while(WiFi.status() != WL_CONNECTED) {
    delay(10);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("Connected to gateway");
  if (requestType == 1) {
    // POST Request
    Serial.println("Sending POST Request");
    postRequest(url, payload);
  }
  else if(requestType == 2) {
    // GET request.
    Serial.println("Sending GET Request");
    getRequest(url);
  }
   Serial.println("Operation Complete!");
  WiFi.disconnect();
}

void setWiFi(const char* Name, const char* Password)
{
  ssid      = Name;
  password  = Password;
  if (! WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("Allocation Error");
  }
  while (!WiFi.softAP(ssid, password)) {
    Serial.print(".");
    delay(200);
  }
  Serial.println("WIFI < " + String(ssid) + " > ... Started");
  Serial.println(WiFi.softAPIP());
  server.begin();
  Serial.println("Server Started");
}

double getDistance(int rssi) {
  int txPower = -59;
  return pow(10, (txPower - rssi)/(100*2));
}

void postRequest(String request, String body) {
  HTTPClient postHttp;
  postHttp.begin(request);
  if(postHttp.POST(body)){
    Serial.println(postHttp.getString());
  }
  else {
    Serial.println("Status return error");
  }
  postHttp.end();
  return;
}

void getRequest(String request){
  HTTPClient http;
  http.begin(request);
  if(http.GET() > 0){
    Serial.println(http.getString());
  }
  else {
    Serial.println("Status return Error!");
  }
  return;
}


void loop() {
  WiFiClient rxClient;
  rxClient = server.available();
  
    if(rxClient){
      if(rxClient.connected()){
        Serial.print("Client connected: ");
        Serial.println(rxClient.remoteIP());
        String message = rxClient.readStringUntil('\n');
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, message);
        
        if(error) {
          Serial.println(error.c_str());
          return;
        }
        
        int distance    = doc["distance"];
        int orientation = doc["orientation"];
        int sensor_id   = doc["sensor_id"];
        int central_orientation = returnAngle();
        int relativeAngle       = 180 - (orientation + central_orientation); 
        
        Serial.print("Distance: " );      
        Serial.println(distance);
        Serial.println("Orientation: " + String(orientation));
        Serial.println("Sensor_id: " + String(sensor_id));
        
        //const char* ssid, const char* password, int requestType, String url, String payload
        //http://192.168.43.10:3000/centrals/1/beacons.json?orientation=23&distance=34&sensor_id=90
               
        String baseUrl = "http://192.168.43.10:3000/centrals";
        baseUrl        = baseUrl + "/" + ROOM_ID + "/beacons.json?";
        //baseUrl = baseUrl + "orientation=23&distance=33&sensor_id=90";
        baseUrl        = baseUrl + "orientation=" + String(orientation) + "&distance=" + String(distance) + "&sensor_id=" + String(sensor_id); 
        Serial.println("Sending: " + String(baseUrl));
        connectWifi("asset", "tracking", POST_REQUEST, baseUrl, "" );
        rxClient.stop();
      } 
    }

    if ((long long)millis() - (long long)timer >= 3000) {
    Serial.print("Station connected: ");
    Serial.println(WiFi.softAPgetStationNum());
    timer = millis();
  } 
}
