/*
 *  This sketch is to be used for diagnostics via wifi
 *  The default sketch is comprised of two parts only. 
 *    1. Initializing WiFi and OTA
 *    2. Initializing TelNet Serial Output/Input
 *  Anything else in this sketch is to be considered temporary for diagnostics only 
 */


/*
 *  MowBot Pins
 *  
 *  Sonar
 *  Ping                                                =   26
 *  Echo                                                =   27
 *  Servo
 *  Pin                                                 =   4
 *  Right Motor
 *  R_Enable  (Forward direction relative to motor)     =   12
 *  R_PWM     (Forward Speed)                           =   13
 *  L_Enable  (Backward direction relative to motor)    =   14
 *  L_PWM     (Backward Speed)                          =   NOT YET CONNECTED
 *  Left Motor
 *  R_Enable  (Forward direction relative to motor)     =   15
 *  R_PWM     (Forward Speed)                           =   NOT YET CONNECTED
 *  L_Enable  (Backward direction relative to motor)    =   16
 *  L_PWM     (Backward Speed)                          =   5
 *  Voltage Sensor
 *  Signal                                              =   35
 *  GPS/Compass
 *  GPS TX                                              =   17
 *  GPS RX                                              =   18
 *  Compass SDA                                         =   21
 *  Compass SDL                                         =   22
 *  
 *  EMPTY PINS
 *  2
 *  0
 *  19
 *  23
 *  25
 *  33
 *  32
 *  34
 *  VN
 *  VP
 *  
 */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "chlabs_bot";
const char* password = "chlabsrobotseverywhere";
char otaHash[] = "ee59085accf685157a4c8cb7d1a76887";

void setup() {
  initOTA();
}

void loop() {
  ArduinoOTA.handle();
  delay(10);
}

void initOTA()  {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname("MowbotCalibration");
  ArduinoOTA.setPasswordHash(otaHash);  
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
      Serial.printf("OTA Update Progress: %u%%\r", (progress / (total / 100)));
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
}
