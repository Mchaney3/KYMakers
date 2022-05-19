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
 *  MowBot ToDo
 *  Motor Control
 *  Voltage Sensor
 *  Servo Calibration - 
 *  Ultrasound
 *  
 */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//const char* ssid = "chlabs_bot";
//const char* password = "chlabsrobotseverywhere";
char otaHash[] = "ee59085accf685157a4c8cb7d1a76887";

/* ------------------------------------------------- */

#include "ESPTelnet.h"          

/* ------------------------------------------------- */

#define SERIAL_SPEED    115200
#define WIFI_SSID       "chlabs_bot"
#define WIFI_PASSWORD   "chlabsrobotseverywhere"

/* ------------------------------------------------- */


ESPTelnet telnet;
IPAddress ip;
uint16_t  port = 23;

/* ------------------------------------------------- */

void setupSerial(long speed, String msg = "") {
  Serial.begin(speed);
//  while (!Serial) {
//  }
  delay(200);  
  Serial.println();
  Serial.println();
  if (msg != "") Serial.println(msg);
}

/* ------------------------------------------------- */

bool isConnected() {
  return (WiFi.status() == WL_CONNECTED);
}

/* ------------------------------------------------- */

bool connectToWiFi(const char* ssid, const char* password, int max_tries = 20, int pause = 500) {
  int i = 0;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  do {
    delay(pause);
    Serial.print(".");
  } while (!isConnected() || i++ < max_tries);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  return isConnected();
}

/* ------------------------------------------------- */

void errorMsg(String error, bool restart = true) {
  Serial.println(error);
  if (restart) {
    Serial.println("Rebooting now...");
    delay(2000);
    ESP.restart();
    delay(2000);
  }
}

/* ------------------------------------------------- */

void setupTelnet() {  
  // passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  
  // passing a lambda function
  telnet.onInputReceived([](String str) {
    // checks for a certain command
    if (str == "ping") {
      telnet.println("> pong");
      Serial.println("- Telnet: pong");
    // disconnect the client
    } else if (str == "bye") {
      telnet.println("> disconnecting you...");
      telnet.disconnectClient();
      }
  });

  Serial.print("- Telnet: ");
  if (telnet.begin(port)) {
    Serial.println("running");
  } else {
    Serial.println("error.");
    errorMsg("Will reboot...");
  }
}

/* ------------------------------------------------- */

// (optional) callback functions for telnet events
void onTelnetConnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" connected");
  telnet.println("\nWelcome " + telnet.getIP());
  telnet.println("(Use ^] + q  to disconnect.)");
}

void onTelnetDisconnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" disconnected");
}

void onTelnetReconnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" reconnected");
}

void onTelnetConnectionAttempt(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" tried to connected");
}

/* ------------------------------------------------- */


/*******      DIAGNOSTIC - DIAGNOSTIC - DIAGNOSTIC    ******/

#include <ESP32Servo.h>
Servo sonarServo;
int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin = 4;

/*******      DIAGNOSTIC - DIAGNOSTIC - DIAGNOSTIC    ******/

void setup() {
  setupSerial(SERIAL_SPEED, "Telnet Test");
  
  Serial.print("- Wifi: ");
  connectToWiFi(WIFI_SSID, WIFI_PASSWORD);
  
  if (isConnected()) {
    ip = WiFi.localIP();
    Serial.println();
    Serial.print("- Telnet: "); Serial.print(ip); Serial.print(" "); Serial.print(port);
    setupTelnet();
  } else {
    Serial.println();    
    errorMsg("Error connecting to WiFi");
  }
  initOTA();

/*******      DIAGNOSTIC - DIAGNOSTIC - DIAGNOSTIC    ******/

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  sonarServo.setPeriodHertz(50);    // standard 50 hz servo
  sonarServo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
  delay(50);
  

/*******      DIAGNOSTIC - DIAGNOSTIC - DIAGNOSTIC    ******/
  
}

void loop() {
  ArduinoOTA.handle();
  telnet.loop();
  
//  telnet.print("I'm ");
//  telnet.println("Alive");

  // send serial input to telnet as output
  if (Serial.available()) {
    telnet.print(Serial.read());
  }

  /*      DIAGNOSTIC    */

    sonarServo.write(25);    // tell servo to go to position in variable 'pos'
    delay(15);             // waits 15ms for the servo to reach the position
}

void initOTA()  {
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
