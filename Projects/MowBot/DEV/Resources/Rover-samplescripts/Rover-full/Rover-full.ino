/* ************  Libraries  ************ */
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SimpleTimer.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DualSonar.h>

/* ************  Blynk setup  ************ */
char auth[] = ""; //auth token
char ssid[] = ""; //wifi credentials
char pass[] = "";
IPAddress ip =IPAddress(192,168,17,110); //only needed when using private Blynk server
/* Blynk virtual pins:
 * V0: autononomous on/off switch
 * V1: heartbeat LED                
 * V2: compass 
 * V3: map
 * V4: LCD screen (two-line)
 * V5: left sonar
 * V6: right sonar
 * V7: joystick (array of two values, range -512 to 512 for each)
 * 
 */
//Objects representing Blynk LCD and map widgets
WidgetLCD lcd(V4);
WidgetMap myMap(V3);
String robotLabel= "R2"; //for labeling rover on the map

/* ************  Timer object  ************ */
SimpleTimer timer; //to control periodic querying sensors and sending  data to Blynk


/* ************  Pins setup  ************ */
static const int RXPin = 5, // the serial connection to GPS. Note that RX, TX
                 TXPin = 4, // refer to 'device view'
                 SDAPin = 0, SCLPin = 2,//I2C pins
                 Motor1aPin = 12, Motor1bPin = 14,
                 Motor2aPin = 13, Motor2bPin = 15;

/* ************  GPS and compass  ************ */
static const float Xoffset=0, Yoffset=0;     // offsets for magnetometer readings                 
static const uint32_t GPSBaud = 9600;        //GPS sensor serial baud rate
static float declinationAngle =-13.2;        //magnetic declination angle
static const double TARGET_LAT=40.893322, TARGET_LNG=-73.130919;    //coordinates of target
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345); // compass object; ID is random
TinyGPSPlus gps;                             // The TinyGPS++ object
SoftwareSerial ss(TXPin, RXPin, 0, 512);     // Serial connection to the GPS module; increase buffer size to 512

/* ************  Sonar  ************ */
DualSonar mySonar(0x11); 

/* ************  Global variables  ************ */
bool stopped=true, 
    heartbeat=true;
int heading = 0; //heading 


/* ***********************************************
 *  Program begins 
 *************************************************/

void setup() {
    pinMode(Motor1aPin, OUTPUT);
    pinMode(Motor1bPin, OUTPUT);
    pinMode(Motor2aPin, OUTPUT);
    pinMode(Motor2bPin, OUTPUT);
    Serial.begin(9600);       // serial connection for debugging
    ss.begin(GPSBaud);
    //start blynk
    Blynk.begin(auth, ssid, pass);
    Serial.println("Activating GPS");
    
    Wire.begin(SDAPin, SCLPin); //I2C bus, for compass and sonar sensors
    /* Initialise the compass sensor */
    if (compass.begin()) {
        Serial.println("Compass started");
    } else  {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Problem starting the compass. Please check connections.");  
    }
    
    //initialize the sonar sensor
    Serial.println("Activating the  Sonar"); Serial.println("");
    //give sensor time to initialize
    delay(500);
    //start pinging
    mySonar.begin();
    //check sensor status
    if (mySonar.isActive())   {   
        //all ok
        Serial.println("Sonar started");
    } else  {
        Serial.println("Problem starting the sonar. Please check connections.");        
    }
    // setup timer which would call updateSensors function every 1000 ms
    // if you are using local blynk srever, you shoudl certainly send data more frequently, 
    // e.g. every 250ms
    // but for blynk's public servers, this will lead to flood errors
    timer.setInterval(1000L, periodicUpdate);
    
}

void loop() {
    timer.run();
    Blynk.run();
    updateGPS();
}


//periodic update function: read compass and sonar sensors, sned data to Blynk, etc
void periodicUpdate() {
    String line1, line2;
    //pulse the LED 
    if (heartbeat) {
        Blynk.virtualWrite(V1, 255);
    } else {
        Blynk.virtualWrite(V1, 0);        
    }
    heartbeat=!heartbeat;
    //get sonars
    mySonar.update();
    // send data to blynk
    //compass
    Blynk.virtualWrite(V2, (int)getHeading());
    //distances
    Blynk.virtualWrite(V5, mySonar.distanceL());
    Blynk.virtualWrite(V6, mySonar.distanceR());
   
    //lcd
    lcd.clear();
    if (gps.location.isValid() && (gps.location.age() < 3000)) {
        //position current
        line1 = String("lat: ") + String(gps.location.lat(), 6);
        line2 = String("lng: ") + String(gps.location.lng(), 6);
        lcd.print(0, 0, line1);
        lcd.print(0, 1, line2);
        //update position on map
        myMap.location(2, gps.location.lat(), gps.location.lng(), robotLabel);
    } else {
        //position is old
        lcd.print(0, 0, "GPS lost");
    }
}


void updateGPS() {
    //read data from serial connection to GPS
    while (ss.available() > 0) {
        gps.encode(ss.read());
    }
}

/* Gets new heading info  from compass 
 * (in degrees, North =0, West=270)
 */
float getHeading() {
    sensors_event_t event; 
    compass.getEvent(&event);
    
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

//code to run when receiving change on V0
BLYNK_WRITE(V0) {
    stopped=param.asInt();  
    //if stopped, grey out the joystick
    if (stopped) {
        Blynk.setProperty(V7, "color", "#000000");
    } else {
        Blynk.setProperty(V7, "color", "#00FF00");       
    }
}


BLYNK_WRITE(V7) {//joystick input from the app 
    int x = param[0].asInt(); //now x,y are between -512 and 512
    int y = param[1].asInt();
    //make deadzone for turns. 
    float powerLeft = (y - x) / 512.0;
    float powerRight = (y + x) / 512.0;
    if (!stopped) {
        setMotors(powerLeft, powerRight);
    }
}
/*
 *  Set motors. Each motor power shoudl be float between -1 and 1
 *  If values are outside of this range, both vlaues will be rescaled:  
 *  e.g., calling setMotors(1.0,2.0) is same as setMotors(0.5, 1.0)
 */
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
    
}

