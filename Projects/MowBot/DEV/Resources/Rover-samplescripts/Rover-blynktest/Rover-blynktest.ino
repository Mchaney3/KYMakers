#include <SimpleTimer.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// Blynk auth token
// FILL IN YOUR DATA! 
char auth[] = "";            //authorization token
char ssid[] = "ubnt-shurik"; // WiFi credentials.
char pass[] = "12af123456";  //WiFi pwd 
IPAddress ip =IPAddress(192,168,17,110); //IP of local blynk server 
                                         //Ignore if using public blynk cloud 

//NodeMCU pins
static const int RXPin = 5, // the serial connection to GPS. Note that RX, TX
                 TXPin = 4, // refer to 'device view'
                 SDAPin = 0, SCLPin = 2,//I2C pins
                 Motor1aPin = 13, Motor1bPin = 15, //right motor
                 Motor2aPin = 12, Motor2bPin = 14;//left motor
/* Blynk virtual pins:
 * V0: motors on/off switch
 * V1: heartbeat LED                
 * V7: joystick (array of two values, range -512 to 512 for each)
 */
 
// timer for updating sensors, sending data to blynk, etc 
SimpleTimer timer;

bool heartbeat=true;
bool stopped=true; //all motors stopped

void setup() {
    pinMode(Motor1aPin, OUTPUT);
    pinMode(Motor1bPin, OUTPUT);
    pinMode(Motor2aPin, OUTPUT);
    pinMode(Motor2bPin, OUTPUT);
    //start blynk
    Blynk.begin(auth, ssid, pass);
    // If using private server:
    // Blynk.begin(auth, ssid, pass, ip);
    timer.setInterval(250L, periodicUpdate); //run function periodicUpdate every 250ms
}

void loop() {
    timer.run();
    Blynk.run();
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

void periodicUpdate(){
    //send data to blynk, reads sensors....
    //pulse the LED 
    if (heartbeat) {
        Blynk.virtualWrite(V1, 255);
    } else {
        Blynk.virtualWrite(V1, 0);        
    }
    heartbeat=!heartbeat;
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



