/* ************  Libraries  ************ */
#include <TinyGPS++.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include <SimpleTimer.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#include <SPI.h>
#include <SdFat.h>

#include <HCSR04.h>

/********************   Voltage Sensor    *********/

const int Analog_channel_pin= 39;
int ADC_VALUE = 0;
float voltage_value = 0.0; 

/********************   Motor Setup       *********/

// Right Motor
int Rt_Motor_Speed = 13; 
int Rt_Motor_IN3 = 12; 
int Rt_Motor_IN4 = 14;

// Left Motor
int Lt_Motor_Speed = 5; 
int Lt_Motor_IN1 = 19; 
int Lt_Motor_IN2 = 2; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannelRtMotor = 0;
const int pwmChannelLtMotor = 0;
const int resolution = 8;
int dutyCycle = 200;
int motorSpeed;

/********************  SD Card Setup  *************/
#define SD_FAT_TYPE 3

// Pin numbers in templates must be constants.
const uint8_t SOFT_MISO_PIN = 23;
const uint8_t SOFT_MOSI_PIN = 16;
const uint8_t SOFT_SCK_PIN  = 25;
//
// Chip select may be constant or RAM variable.
const uint8_t SD_CS_PIN = 32;

// SdFat software SPI template
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)

// File system object.
SdFs sd;

// Use for file creation in folders.
FsFile file;

// Directory file.
FsFile root;


//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))
//------------------------------------------------------------------------------


/* ************  Blynk setup  ************ */
char auth[] = "6f1da165f66a4fd8871543f6b65a9dc4"; //auth token
char ssid[] = "chlabs_bot"; //wifi credentials
char pass[] = "chlabsrobotseverywhere";
char blynkServer[] = "192.168.1.249";
const int blynkPortHTTP = 8181;

//IPAddress ip =IPAddress(192,168,17,110); //only needed when using private Blynk server
/* Blynk virtual pins:
 * V0: autononomous on/off switch
 * V1: heartbeat LED                
 * V2: compass 
 * V3: map
 * V4: LCD screen (two-line)
 * V5: left sonar
 * V6: right sonar
 * V7: joystick (array of two values, range -512 to 512 for each)
 * v8: left Motor Output
 * v9: Right Motor Output
 * 
 */
 
//Objects representing Blynk LCD and map widgets
WidgetLCD lcd(V4);
WidgetMap myMap(V3);

/* ************  OTA Setup     ************ */
char robotLabel[] = "MowBot"; //for labeling rover on the map
char otaHash[] = "ee59085accf685157a4c8cb7d1a76887";

/* *********** WebSerial Setup ************ */

AsyncWebServer server(80);

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

/* ************  Timer object  ************ */

SimpleTimer timer; //to control periodic querying sensors and sending  data to Blynk

/* ************  GPS and compass  ************ */

static const int RXPin = 18, // the serial connection to GPS. Note that RX, TX
                 TXPin = 17, // refer to 'device view'
                 SDAPin = 21, 
                 SCLPin = 22; //I2C pins
                 
static const float Xoffset=20, Yoffset=-97;     // offsets for magnetometer readings                 
static const uint32_t GPSBaud = 9600;        //GPS sensor serial baud rate
//The angle in radians is equal to the degrees multiplied by 0.017453.
//my declination is -5 degrees 45 minutes or 5.75 degrees. -5.75 * 0.017453
static float declinationAngle = -0.100356;        //magnetic declination angle
static const double TARGET_LAT=37.890482, TARGET_LNG=-84.561531;    //coordinates of target
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // compass object; ID is random
TinyGPSPlus gps;         // The TinyGPS++ object

/* ************  Sonar  ************ */

HCSR04 sonar(26, 27); //initialisation class HCSR04 (trig pin , echo pin)

/* ************  Global variables  ************ */
bool autonomous=true, 
    heartbeat=true;
float distance=1000,  ///distance to target, in meters
      Kp=0.5,         // coefficient  for proportional steering
      course=0;       //course to target, in degrees; North=0, West=270

/* ***********************************************
 *  Program begins 
 *************************************************/

void setup() {

  Serial.begin(115200);
  
  delay(1000);

    /************ Blynk Init which also start WiFi in STA mode *********/
  
  Serial.println("Starting Blynk and enabling OTA");
  Blynk.begin(auth, ssid, pass, blynkServer, blynkPortHTTP);
  // put MowBot on the map
  myMap.location(1, gps.location.lat(), gps.location.lng(), robotLabel);
  //myMap.location(2, TARGET_LAT, TARGET_LNG, "Waypoint 1");

  /*************** Init OTA *****************/
    
  initOTA();

  /************** Init Web Serial **************/
  
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);
  /* Attach Message Callback */
  WebSerial.msgCallback(recvMsg);
  server.begin();

  Serial.println("MowBot MowBoot SUCCESS!");
  Serial.print("Connect to WebSerial at: ");  Serial.print(WiFi.localIP());   Serial.println("/webserial");
  delay(3000);

/********   Initialize Motors   **********/

  initMotors();

  delay(1000);

  /***************** SD CARD INIT***********/

  initSD();

  delay(1000);
  
  /****************** GPS and Compass Init  ********************/
    
  Serial.println("Activating GPS");
  Serial2.begin(GPSBaud);        //software serial connection to

  delay(1000);
  
  /* Initialise the compass sensor */
  if (mag.begin()) {
      Serial.println("Compass started");
  } else  {
    /* There was a problem detecting the magentometer ... check your connections */
    Serial.println("Problem starting the compass. Please check connections.");  
  }
  
  delay(1000);
  
  /****** Run timer for periodicUpdate() - "Scheduled Task"   *******/
  
  // setup timer which would call updateSensors function every 1000 ms
  // if you are using local blynk srever, you shoudl certainly send data more frequently, 
  // e.g. every 250ms
  // but for blynk's public servers, this will lead to flood errors
  timer.setInterval(1000L, periodicUpdate);
  
}

void loop() {

  /*********  Check for OTA Update  *********/
  
  ArduinoOTA.handle();

  /*********  Run Periodic Updates  *********/
  
  timer.run();//update sensors and send info to cellphone

  /*********  Phone home To Blynk Server  *********/
  
  Blynk.run();

  /*********  Get latest GPS info  *********/
  
  updateGPS(); //read messages from GPS sensor

  /*********  Process Waypoint IF Autonomous  *********/
  
  if (autonomous && gps.location.isValid() && (gps.location.age() <3000 ) && (distance > 1)  ) {
    //we are in autonomous mode, and have valid location fix
    //get distance to target, in meters
    distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),TARGET_LAT, TARGET_LNG);
    //get course to target, in degrees; North=0, West=270
    course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(),TARGET_LAT, TARGET_LNG);
    //compare with current heading, and determine error. 
    //If error>0, we need to turn right; if <0, left
    float error=course-getHeading();
    if (error > 8)  {
      turnRight();
    }

    else if (error < -8)  {
      turnLeft();
    }

    else  {
      motorsForward();
    }
    
    
    //normalize so that error is between -180 and 180
    if (error > 180 ) error-=360;
    if (error < -180 ) error+=360;
    //set course 
    //setMotors (60+Kp*error, 60-Kp*error);
    setMotors (60+Kp*error, 60-Kp*error);
    //Serial.print("LeftM: "); Serial.print(60+Kp*error); Serial.print("   RightM: "); WebSerial.println(60-Kp*error);
  }
  //motorsBackward();
}

void initMotors() {
  // sets the Right motor pins as outputs:
  pinMode(Rt_Motor_IN3, OUTPUT);
  pinMode(Rt_Motor_IN4, OUTPUT);
  pinMode(Rt_Motor_Speed, OUTPUT);
  
  // sets the Left motor pins as outputs:
  pinMode(Lt_Motor_IN1, OUTPUT);
  pinMode(Lt_Motor_IN2, OUTPUT);
  pinMode(Lt_Motor_Speed, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannelRtMotor, freq, resolution);
  ledcSetup(pwmChannelLtMotor, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Rt_Motor_Speed, pwmChannelRtMotor);
  ledcAttachPin(Lt_Motor_Speed, pwmChannelLtMotor);
  motorsStop();
  Serial.println("Motors Initialized");
  WebSerial.println("Motors Initialized");
}

void initSD() {
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("Failed to init SD card. Check wiring.");
    WebSerial.println("Failed to init SD card. Check wiring.");
    sd.initErrorHalt();
  }
  else  {
  if (!file.open("SoftSPI.txt", O_RDWR | O_CREAT)) {
    Serial.println("Failed to open SD boot log. Check SD card!");
    WebSerial.println("Failed to open SD boot log. Check SD card!");
    sd.errorHalt(F("open failed"));
  }
  file.println(F("This line was printed using software SPI."));

  file.rewind();

  while (file.available()) {
    Serial.write(file.read());
    //WebSerial.print(file.read());
  }

  file.close();

  Serial.println(F("SD card initialization complete!"));
  WebSerial.println(F("SD card initialization complete!"));
  }
}

void initOTA()  {
  ArduinoOTA.setHostname(robotLabel);
//  ArduinoOTA.setPasswordHash(otaHash);  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      WebSerial.println("Start updating " + type);
    })
    .onEnd([]() {
      WebSerial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      WebSerial.print("OTA Update Progress: ");
      WebSerial.print(progress / (total / 100));
    })
    .onError([](ota_error_t error) {
      WebSerial.print("Error[%u]: ");
      WebSerial.print(error);
      if (error == OTA_AUTH_ERROR) WebSerial.println(" Auth Failed");
      else if (error == OTA_BEGIN_ERROR) WebSerial.println(" Begin Failed");
      else if (error == OTA_CONNECT_ERROR) WebSerial.println(" Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) WebSerial.println(" Receive Failed");
      else if (error == OTA_END_ERROR) WebSerial.println(" End Failed");
    });
  ArduinoOTA.begin();
}

float mainBatteryVoltage() {
  ADC_VALUE = analogRead(Analog_channel_pin);
  voltage_value = (ADC_VALUE * 3.3 ) / (790);
  return(voltage_value);
}

/******   periodic update function: read compass and sonar sensors, sned data to Blynk, etc   ******/

void periodicUpdate() {
  String line1, line2;
  //pulse the LED 
  if (heartbeat) {
      Blynk.virtualWrite(V1, 255);
  } else {
      Blynk.virtualWrite(V1, 0);        
  }
  heartbeat=!heartbeat;
  
  /******   Write Debug to WebSerial    ********/
  
  Serial.print("Distance To Object = ");  Serial.println(sonar.dist()); // return curent distance in serial
  WebSerial.print("ObjDistance  = ");     WebSerial.println(sonar.dist());
  WebSerial.print("Heading      = ");     WebSerial.println(getHeading());
  WebSerial.print("Voltage      = ");     WebSerial.println(mainBatteryVoltage());
  
  /******   Send data to blynk    *****/
  
  Blynk.virtualWrite(V13, mainBatteryVoltage());  //  Main system supply voltage
  Blynk.virtualWrite(V2, (int)getHeading());      //  compass
  Blynk.virtualWrite(V5, sonar.dist());           //  distance from object via Sonar
  
  lcd.clear();    //lcd and map
  if (gps.location.isValid() && (gps.location.age() < 3000)) {
    line1 = String("lat: ") + String(gps.location.lat(), 6);
    line2 = String("lng: ") + String(gps.location.lng(), 6);
    lcd.print(0, 0, line1);
    lcd.print(0, 1, line2);
    myMap.location(1, gps.location.lat(), gps.location.lng(), robotLabel);  //  update MowBot position on map
    myMap.location(2, TARGET_LAT, TARGET_LNG, "Waypoint 1");                //  update current waypoint position on map
  } else {
    lcd.print(0, 0, "GPS lost");    //position is old
  }
}


void updateGPS() {
  
  /*********    read data from serial connection to GPS   *******/
  
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
}

/***********     Gets new heading info  from compass.     *********/

float getHeading() {
  sensors_event_t event; 
  mag.getEvent(&event);
  float X,Y;
  
  /* 
  *  Get x,y components of magnetic field vector; values are in micro-Tesla (uT))
  *  and correct for offset 
  */
  
  X=event.magnetic.x-Xoffset;
  Y=event.magnetic.y-Yoffset;
  float heading = atan2(-X, -Y) * 180/PI;
  
  //correct for declination angle 
  heading += declinationAngle;
  
  // correct as needed to guarantee that result is between 0 and 360
  if (heading < 0) heading += 360;
  if (heading > 360 ) heading -= 360;
  return(heading);
}

BLYNK_WRITE(V0) {
  autonomous=param.asInt();  
  //if stopped, grey out the joystick
  if (autonomous) {
    Blynk.setProperty(V7, "color", "#000000");
    WebSerial.println("Autonomous Mode Enabled");
    Serial.println("Autonomous Mode Enabled");
  } else {
    Blynk.setProperty(V7, "color", "#00FF00");
    WebSerial.println("Autonomous Mode Disabled");    
    Serial.println("Autonomous Mode Disabled"); 
  }
}

BLYNK_WRITE(V7) {     
  /*    TODO: Create 4 buttons in Blynk for forward, back, left, right + 2 for rotate left, rotate right 
   *    To use joystick, set joystick in app to send between -255, 255. If negative, set motor in backward direction
   */
  int x = param[0].asInt(); //now x,y are between -255 and 255
  int y = param[1].asInt();
  //make deadzone for turns. 
  float powerLeft;
  float powerRight;
  if (!autonomous) {
    setMotors(powerLeft, powerRight);
  }
}

BLYNK_WRITE(V10) {  // Forward / Backward
  motorSpeed = param[0].asInt();
  if (motorSpeed == -1)  {    //    if -200
    motorsForward();
    ledcWrite(pwmChannelRtMotor, dutyCycle);   
    ledcWrite(pwmChannelLtMotor, dutyCycle);
  }
  else if (motorSpeed == 1) {
    motorsBackward();
    ledcWrite(pwmChannelRtMotor, dutyCycle);   
    ledcWrite(pwmChannelLtMotor, dutyCycle);
  }
  Blynk.virtualWrite(V8, motorSpeed);
  Blynk.virtualWrite(V9, motorSpeed);
}

BLYNK_WRITE(V11) {  // Left / Right
  
}

BLYNK_WRITE(V12) {  // STOP
    
    motorsStop();
    Blynk.virtualWrite(V8, 0);
    Blynk.virtualWrite(V9, 0);
}


/*
 *  Set motors. Each motor power shoudl be float between -1 and 1
 *  If values are outside of this range, both vlaues will be rescaled:  
 *  e.g., calling setMotors(1.0,2.0) is same as setMotors(0.5, 1.0)
 */




 /*******   MY MOTOR SOLUTION
  
  *    Start with basic dumb forward back left right controls using values of 200 - 255
  *    Write function for left motor forward, right motor forward, left motor backward, right motor backward
*/

void motorsStop() {
  setMotors(0,0);
  digitalWrite(Lt_Motor_IN1, LOW);
  digitalWrite(Lt_Motor_IN2, LOW);
  digitalWrite(Rt_Motor_IN3, LOW);
  digitalWrite(Rt_Motor_IN4, LOW);
  Serial.println("Motors stopped");
}

void motorsForward() {
  digitalWrite(Lt_Motor_IN1, LOW);
  digitalWrite(Lt_Motor_IN2, HIGH);
  digitalWrite(Rt_Motor_IN3, HIGH);
  digitalWrite(Rt_Motor_IN4, LOW);
  Serial.println("Moving Forward");
  setMotors(dutyCycle, dutyCycle);
}

void motorsBackward() {
  digitalWrite(Lt_Motor_IN1, HIGH);
  digitalWrite(Lt_Motor_IN2, LOW); 
  digitalWrite(Rt_Motor_IN3, LOW);
  digitalWrite(Rt_Motor_IN4, HIGH);
  Serial.println("Moving Backwards");
  setMotors(dutyCycle, dutyCycle);
}

void turnRight() {
  digitalWrite(Lt_Motor_IN1, LOW);
  digitalWrite(Lt_Motor_IN2, HIGH); 
  digitalWrite(Rt_Motor_IN3, LOW);
  digitalWrite(Rt_Motor_IN4, LOW);
  Serial.println("Turning Right");
  setMotors(dutyCycle, dutyCycle);
}

void turnLeft() {
  digitalWrite(Lt_Motor_IN1, LOW);
  digitalWrite(Lt_Motor_IN2, LOW); 
  digitalWrite(Rt_Motor_IN3, HIGH);
  digitalWrite(Rt_Motor_IN4, LOW);
  Serial.println("Turnin Layuft");
  setMotors(dutyCycle, dutyCycle);
}


void setMotors(float left, float right) {
  
  ledcWrite(pwmChannelRtMotor, right);   
  ledcWrite(pwmChannelLtMotor, left);
  Blynk.virtualWrite(V8, left);
  Blynk.virtualWrite(V9, right);

    /*
  
    //compute the max of two numbers. Unfortunately, 
    //usual max macro doesn't work: https://github.com/esp8266/Arduino/issues/398
    // so we make our own
    float m = abs(left);
    if (abs(right)>m) m=abs(right);
    //rescale if needed
    if (m > 1) {
        left = left / m;
        right = right / m;
    }

  if (left <= 0) {
    left = left*-1;
  }

  if (right <= 0) {
    right = right*-1;
  }
  Serial.print("RAW Left Motor: "); Serial.print(left); Serial.print(" - RAW Right Motor: "); WebSerial.println(right);
  int mappedleft = map(left, 0, 0, 1, 255);
  int mappedright = map(right, 0, 0, 1, 255);

  Serial.print("MAPPED Left Motor: "); Serial.print(mappedleft); Serial.print(" - MAPPED Right Motor: "); WebSerial.println(mappedright);


/*  Rewriting this section to use BTS7960 Driver. Should be able to analogWrte 0 - 255 as values, not 0 - 1023    
 *  Plus we're using an ESP32, not the 8266
 *  
    // set left motor. Note that for ESP8266, analogWrite expects parameter to be 0-1023:
    // http://esp8266.github.io/Arduino/versions/2.0.0/doc/reference.html#analog-output
    // unlike the usual Arduino, whihc expects range of 0-255


    if (left > 0) {
        analogWrite(Motor1aPin, 0);
        analogWrite(Motor1bPin, left * 1023);
    } else {
        analogWrite(Motor1aPin, -left * 1023);
        analogWrite(Motor1bPin, 0);
    }
    //right
    if (right > 0) {
        analogWrite(Motor2aPin, 0);
        analogWrite(Motor2bPin, right * 1023);
    } else {
        analogWrite(Motor2aPin, -right * 1023);
        analogWrite(Motor2bPin, 0);
    }
    

    //Serial.print("Left: "); Serial.print(left); Serial.print("\nRight: "); WebSerial.println(right);
    
    */
}
