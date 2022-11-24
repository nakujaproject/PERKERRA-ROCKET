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
#include <EEPROM.h>


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
int angleX, angleY, prevangleX = 90, prevangleY = 90;

unsigned long currentTime, previousTime = 0;
double elaspedTime, error, lastError = 0, cumError = 0, rateError, PID;
const double setPoint = 0;
float Kp = 10, Kd = 1, Ki = 0;

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
void MOTORSetup();
void MotorWrite(int a, int b);

void setup() {
  Serial.begin(9600);

  EEPROM.begin(1);
  int resets = EEPROM.read(0);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }


  if (resets != 1) {
    EEPROM.write(0,1);
    EEPROM.commit();
    delay(100);
    resets = EEPROM.read(0);
    Serial.print("resets: ");
    Serial.println(resets);
    delay(2000);
    ESP.restart();
  }

  EEPROM.write(0,0);
  EEPROM.commit();
  delay(2000);
  MPUSetup();
  MOTORSetup();

  SPIClass spiSD = SPIClass(HSPI); // Neither HSPI nor VSPI seem to work
  spiSD.begin(13, 15, 4, 5);

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect, spiSD)) {
    Serial.println("initialization failed!");
    //while (1);
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
  if (y < 0){
    leds[0] = CRGB::Red;
    FastLED.show();
  }else{
    leds[0] = CRGB::Green;
    FastLED.show();
  }
  error = setPoint - y;
  if(abs(error - lastError) < 5){          
MotorWrite(0, 0);
  }
  else {
  
  }
  rateError = (error - lastError) / elaspedTime;
  cumError += error * elaspedTime;
  PID = Kp * error + Kd * rateError + Ki * cumError;


  angleX = ANGLE + PID;
  angleY = ANGLE - PID;


   angleX = constrain(angleX, 0, 30);
   angleY = constrain(angleY, 0, 30);

   /*myFile = SD.open("/test.txt", FILE_APPEND);

  if (myFile) {
    myFile.print("error: ");
    myFile.print(error);
    myFile.print(" ,anglex: ");
    myFile.print(angleX);
    myFile.print(" ,angley: ");
    myFile.println(angleY);
  }
  myFile.close();*/

    prevangleX =  angleX;
  prevangleY = angleY;   
  
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


  lastError = error;
   previousTime = currentTime;

   ArduinoOTA.handle();
}

void MOTORSetup() {
  Servo1.setPeriodHertz(50);
  Servo2.setPeriodHertz(50);
  Servo3.setPeriodHertz(50);
  Servo4.setPeriodHertz(50);

  Servo1.attach(servopin_1, 500, 2400);
  Servo2.attach(servopin_2, 500, 2400);
  Servo3.attach(servopin_3, 500, 2400);
  Servo4.attach(servopin_4, 500, 2400);
}

void MotorWrite(int a, int b){
  Servo1.write(a);
  Servo2.write(a);
  Servo3.write(b);
  Servo4.write(b);  
}

void MPUSetup() {
  Wire.setClock(400000);
  Wire.begin();
  delay(10);
  Wire.beginTransmission(0x69);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void getAngle(double &A, double &B, double &C) {
  int16_t Gyro_X, Gyro_Y, Gyro_Z;// Ac_X, Ac_Y, Ac_Z;
  // int minV = 265;
  // int maxV = 402;
  Wire.beginTransmission(0x69);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x69, 6);

  Gyro_X = Wire.read() << 8 | Wire.read();
  Gyro_Y = Wire.read() << 8 | Wire.read();
  Gyro_Z = Wire.read() << 8 | Wire.read();

 

  // A = RAD_TO_DEG * (atan2(-Gyro_Y, -Gyro_Z) + PI);
  // B = RAD_TO_DEG * (atan2(-Gyro_X, -Gyro_Z) + PI);
  // C = RAD_TO_DEG * (atan2(-Gyro_Y, -Gyro_X) + PI);
A = Gyro_X / 131.0;
B = Gyro_Y / 131.0;
C = Gyro_Z / 131.0;


  
}
