#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* ************  Pins setup  ************ */
static const int RXPin = 18, // the serial connection to GPS. Note that RX, TX
                 TXPin = 17, // refer to 'device view'
                 SDAPin = 21, SCLPin = 22,//I2C pins
                 Motor1aPin = 12, Motor1bPin = 13,
                 Motor2aPin = 14, Motor2bPin = 15;

/* ************  GPS and compass  ************ */
static const float Xoffset=20, Yoffset=-97;     // offsets for magnetometer readings                 
static const uint32_t GPSBaud = 9600;        //GPS sensor serial baud rate
//The angle in radians is equal to the degrees multiplied by 0.017453.
//my declination is -5 degrees 45 minutes or 5.75 degrees. -5.75 * 0.017453
static float declinationAngle = -0.100356;        //magnetic declination angle
static const double TARGET_LAT=37.890482, TARGET_LNG=-84.561531;    //coordinates of target
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // compass object; ID is random
TinyGPSPlus gps;         // The TinyGPS++ object

/* ************  Global variables  ************ */
bool autonomous=true, 
    heartbeat=true;
float distance=1000,  ///distance to target, in meters
      Kp=0.5,         // coefficient  for proportional steering
      course=0;       //course to target, in degrees; North=0, West=270
      
void setup() {
  //set up motor pins
  pinMode(Motor1aPin, OUTPUT);
  pinMode(Motor1bPin, OUTPUT);
  pinMode(Motor2aPin, OUTPUT);
  pinMode(Motor2bPin, OUTPUT);
  
  Serial.begin(115200);       // serial connection for debugging
  delay(500);
  Serial2.begin(GPSBaud);        //software serial connection to GPS
  delay(500);
  Serial.println("Booting...");
  //Wire.begin(SDAPin, SCLPin); //I2C bus, for compass and sonar sensors
  /* Initialise the compass sensor */
  if(!mag.begin()) {
    Serial.println("Problem starting the compass. Please check connections.");
  } else  {
    /* There was a problem detecting the magentometer ... check your connections */
    Serial.println("Compass Initialized");  
  }
}

void loop() {
  updateGPS(); //read messages from GPS sensor
  //if (autonomous && gps.location.isValid() && (gps.location.age() <3000 ) && (distance > 1)  ) {
  if (autonomous && (distance > 1)  ) {
    //we are in autonomous mode, and have valid location fix
    //get distance to target, in meters
    distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),TARGET_LAT, TARGET_LNG);
    //get course to target, in degrees; North=0, West=270
    course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(),TARGET_LAT, TARGET_LNG);
    //compare with current heading, and determine error. 
    //If error>0, we need to turn right; if <0, left
    float error=course-getHeading();
    //normalize so that error is between -180 and 180
    if (error > 180 ) error-=360;
    if (error < -180 ) error+=360;
    //set course 
    setMotors (60+Kp*error, 60-Kp*error);
    Serial.print("LeftM: "); Serial.print(60+Kp*error); Serial.print("   RightM: "); Serial.println(60-Kp*error);
  }
}

void updateGPS() {
  //read data from serial connection to GPS
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
}

// Gets new heading info  from compaSerial2. 
float getHeading() {
  sensors_event_t event; 
  mag.getEvent(&event);
  
  float X,Y;
  // Get x,y components of magnetic field vector; values are in micro-Tesla (uT))
  // and correct for offset 
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

void setMotors(float left, float right) { 
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
  //Blynk.virtualWrite(V8, left);
  //Blynk.virtualWrite(V9, right);
  //Serial.print("Left: "); Serial.print(left); Serial.print("\nRight: "); Serial.println(right);
}
