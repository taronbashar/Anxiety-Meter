
// Import required libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <DFRobot_LIS2DH12.h>

MAX30105 particleSensor;
DFRobot_LIS2DH12 LIS; //Accelerometer
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
long lastBeat1 = 0;
float BPM;
int ABPM;
float GSR;
int AGSR;
int event = 10000;
float bpm;
int avebpm;
float avegsr;
float curgsr;
double a;
String accelo;
String state;
long ir;

// Replace with your network credentials
const char* ssid = "Enter WIFI name";
const char* password = "Enter password";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Auxiliar variables to store the current output state
String output5State = "off";
String output4State = "off";

// Assign output variables to GPIO pins
const int output5 = 5;
const int output4 = 4;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;
int serverStopCount = 0;
// Set your Static IP address
IPAddress local_IP(172, 20, 10, 8);
// Set your Gateway IP address
IPAddress gateway(172, 20, 10, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

String readHeartRate() {
  ir = particleSensor.getIR();

   if (ir > 50000)
  {
    //We sensed a beat!
   // WiFi.mode(WIFI_STA);
    //WiFi.begin(ssid, password);
    serverStopCount = 0;
    long delta = millis() - lastBeat;
    lastBeat = millis();

    BPM = 60 / (delta / 1000.0);

    if (BPM < 255 && BPM > 20)
    {
      rates[rateSpot++] = (byte)BPM; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      ABPM = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        ABPM += rates[x];
      ABPM /= RATE_SIZE;
    }
  }
  else {
    BPM = 0;
    ABPM = 0;
  }
  if (ir < 50000) {
    Serial.println(" No finger?");
    ESP.deepSleep(10e6);
    }
  else {
    Serial.print("BPM=");
    Serial.println(BPM);
  }
  return String(BPM);
}

String readBPM(float beats) { 
  bpm = beats;
  return String(bpm);
}

String readLIS2DHAcceleration() {
  int16_t x, y, z;  
  const float alpha = 0.1;
  double data_filteredx[] = {0, 0, 0};
  double data_filteredy[] = {0, 0, 0};
  double data_filteredz[] = {0, 0, 0};
  const int n = 1;
 
  delay(100);
  LIS.readXYZ(x, y, z);
  LIS.mgScale(x, y, z);

  double da_x = (double)x;
  double da_y = (double)y;

  // Low Pass Filter
  data_filteredx[n] = alpha * da_x + (1 - alpha) * data_filteredx[n-1];

  // Store the last filtered data in data_filtered[n-1]
  data_filteredx[n-1] = data_filteredx[n];

  // Low Pass Filter
  data_filteredy[n] = alpha * da_y + (1 - alpha) * data_filteredy[n-1];

  // Store the last filtered data in data_filtered[n-1]
  data_filteredy[n-1] = data_filteredy[n];
  
  // Print Data
  a = sqrt((data_filteredx[n]*data_filteredx[n]) + (data_filteredy[n]*data_filteredy[n]));
  
  if (isnan(a)) {    
    Serial.println("Failed to read from LIS2DH sensor!");
    return "";
  }
  else {
    return String(a);
  }
}

String analyzeAcceleration(double b) {
  if (b > 0.00 && b < 75.00) {
    state = "Idle";
  }
  else if (b > 75.00 && b < 200.00) {
    state = "Walking/Jogging";
  }
  else if (b > 200.00) {
    state = "Running";
  }
  return state;
}

String GSRamplitude() {
  int g = analogRead(0);
  long delta1 = millis() - lastBeat1;
  lastBeat1 = millis();

  curgsr = 60 / (delta1 / 1000.0);

  rates[rateSpot++] = (byte)GSR; //Store this reading in the array
  rateSpot %= RATE_SIZE; //Wrap variable
   //Take average of readings
  AGSR = 0;
  for (byte y = 0 ; y < RATE_SIZE ; y++)
    AGSR += rates[y];
  AGSR /= RATE_SIZE;
  avegsr = AGSR;
  curgsr = g;
  return String(g);
}

String Anxiom(String accelo, int curgsr, float avegsr, float bpm, int ABPM) {
  String anxiom;
  if (accelo == "Idle") {
    anxiom = "Anxiety detection in progress...";
  }
  if (accelo == "Idle" && bpm > (1.25*ABPM) && bpm < (1.7*ABPM) && curgsr > (1.15*avegsr)) {
    anxiom = "Anxiety detected.  Potential panic attack imminent.  Be cautious.";
  }
  else if (accelo == "Idle" && bpm > (1.7*ABPM) && curgsr > (1.5*avegsr)) {
    anxiom = "Anxiety detected.  Panic attack present.  Please calm down.";
  }
  else if (accelo == "Walking/Jogging" && bpm > (1.7*ABPM) && curgsr > (1.15*avegsr)) {
    anxiom = "Increase in physical activity detected.  Somewhat unreliable data analysis.  Return to idle movement for optimal anxiety detection";
  }
  else if (accelo == "Running" && bpm > (1.7*ABPM) && curgsr > (1.15*avegsr)) {
    anxiom = "Due to increased physical activity, unable to determine if anxiety/panic attack is present";
  }
  return String(anxiom);
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  while(!Serial);
  delay(100);
  
  // Initialize sensor
  if (!particleSensor.begin(Wire, 100000)) {//Use default I2C port, 400kHz speed
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  
  Serial.println("Place your index finger on the sensor with steady pressure.");
  
  while(LIS.init(LIS2DH12_RANGE_16GA) == -1){                            //Equipment connection exception or I2C address error
     Serial.println("No I2C devices found");
     delay(1000);   
    }
    
  // Initialize SPIFFS
  if(!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
    }

  pinMode(output5, OUTPUT);
  pinMode(output4, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output5, LOW);
  digitalWrite(output4, LOW);
  
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  byte ledBrightness = 150; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 1000; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 118; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  
  server.on("/heartrate", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readHeartRate().c_str());
  });

  server.on("/BPM", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readBPM(BPM).c_str());
  });

   server.on("/acceleration", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readLIS2DHAcceleration().c_str());
  });
  
  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", analyzeAcceleration(a).c_str());
  });

  server.on("/GSR", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", GSRamplitude().c_str());
  });

  server.on("/Anxiom", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", Anxiom(state,curgsr,avegsr,BPM,ABPM).c_str());
  });
    
  // Start server
  server.begin();
}
 
void loop() {
}
