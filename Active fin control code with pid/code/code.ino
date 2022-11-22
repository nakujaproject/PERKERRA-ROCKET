#include <Wire.h>
#include <ESP32Servo.h>
#include "FastLED.h"
#define ANGLE 90
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


const int chipSelect = 5;
File myFile;

const char* ssid = "sammy2";
const char* password = "12345678";

SPIClass spiSD(HSPI);

#define DATA_PIN    14

#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    2
#define BRIGHTNESS  255

CRGB leds[NUM_LEDS];

double x, y, z;
int angleX, angleY;

unsigned long currentTime, previousTime = 0;
double elaspedTime, error, lastError = 0, cumError = 0, rateError, PID;
const double setPoint = 0;
float Kp = 100, Kd = 0, Ki = 0;

int servopin_1 = 27;  //initializing the pins
int servopin_2 = 26;
int servopin_3 = 25;
int servopin_4 = 32;

Servo Servo1;  // creating servo objects
Servo Servo2;
Servo Servo3;
Servo Servo4;


void getAngle(double &A, double &B, double &C);
void MPUSetup();

void setup() {
  Serial.begin(9600);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  
  MPUSetup();
  MOTORSetup();

  SPIClass spiSD = SPIClass(HSPI); // Neither HSPI nor VSPI seem to work
  spiSD.begin(13, 15, 4, 5);

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect, spiSD)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");


  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
    .setCorrection(TypicalLEDStrip)
    .setDither(BRIGHTNESS < 255);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  currentTime = millis();
  elaspedTime = currentTime - previousTime;
  getAngle(x, y, z);  //update yaw, pitch and roll
  if (z < 0){
    leds[0] = CRGB::Red;
    FastLED.show();
  }else{
    leds[0] = CRGB::Green;
    FastLED.show();
  }
  error = setPoint - z;
  rateError = (error - lastError) / elaspedTime;
  cumError += error * elaspedTime;
  PID = Kp * error + Kd * rateError + Ki * cumError;


  angleX = ANGLE + PID;
  angleY = ANGLE - PID;


   angleX = constrain(angleX, 0, 180);
   angleY = constrain(angleY, 0, 180);

   myFile = SD.open("/test.txt", FILE_WRITE);

  if (myFile) {
    myFile.print("error: ");
    myFile.print(error);
    myFile.print(" ,anglex: ");
    myFile.print(angleX);
    myFile.print(" ,angley: ");
    myFile.println(angleY);
  }
  myFile.close();


   MotorWrite(angleX, angleY);

   Serial.print(x);
   Serial.print(" ");
   Serial.print(y);
   Serial.print(" ");
   Serial.print(z);
   Serial.print(" Error = ");
   Serial.print(error);
   Serial.print(" proportional = ");
   Serial.print( Kp * error);
   Serial.print("  differential = ");
   Serial.print(Kd * rateError);
   Serial.print("  integral = ");
   Serial.print(Ki * cumError);
   Serial.print("  angleX = ");
   Serial.print(angleX);
   Serial.print("  angleY = ");
   Serial.print(angleY);
   Serial.print("  Error = ");
   Serial.print(error);
   Serial.print(" PID = ");  
   Serial.println(PID);


   previousTime = currentTime;

   ArduinoOTA.handle();
}
