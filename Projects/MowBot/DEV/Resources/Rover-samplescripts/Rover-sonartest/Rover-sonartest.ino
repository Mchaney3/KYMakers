/*
 * Test ultrasonic (sonar) sensors
 * Prints  readings to serial monitor (you need to open serial monitor at 9600 Baud)
 */
 
// I2C library
#include <Wire.h>
// DualSonar library by Alexander Kirillov, see https://github.com/shurik179/DualSonar
#include <DualSonar.h>
static const int RXPin = 5, // the serial connection to GPS. Note that RX, TX
                 TXPin = 4, // refer to 'device view'
                 SDAPin = 0, SCLPin = 2,//I2C pins
                 Motor1aPin = 12, Motor1bPin = 14,
                 Motor2aPin = 13, Motor2bPin = 15;


/* Create new sonar object using I2C address 17=0x11 (this is the default)
 * The following functions/properties are avaialble:
 * void mySonar.begin()
 * void mySonar.stop()
 * bool mySonar.isActive();
 * void mySonar.update() -- updates the sensor values (stored in internal variables)
 * int mySonar.distanceL(), mySonar.distanceR()
 * Return the left and right distances received at last update
 *         Values are in mm. Maximal distance is about 3444 mm (3.444 m);
 *         if the distance is larger than that, or if no echo was received, 
 *          the value of 3444 is returned.    
 */
DualSonar mySonar(0x11); 

void setup() {
    //start I2C 
    Wire.begin(SDAPin, SCLPin);
    //start serial monitor 
    Serial.begin(9600);
    Serial.println("Dual Sonar Test"); Serial.println("");
    //give sensor time to initialize
    delay(500);
    //start pinging
    mySonar.begin();
    //check sensor status
    if (mySonar.isActive())   {   //all ok
        Serial.println("Sonar started");
        Serial.println("Type 's' to stop the sensor, 'r' to restart");
    } else {
        Serial.println("Problem starting the sonar. Please check connections.");        
    }
}


void loop() {   
    //read user input 
    char c=readChar();
    if (c=='r') {
        mySonar.begin();
        Serial.println("Sonar restarted. Enter 's' to stop");
    } else if (c=='s') {
        mySonar.stop();
        Serial.println("Sonar stopped. Enter 'r' to restart");
    }        
    if (mySonar.isActive()) {
        //update sensor readings
        mySonar.update();
        Serial.print("Left distance (mm): ");
        Serial.print(mySonar.distanceL());
        Serial.print("  Right distance (mm): ");
        Serial.println(mySonar.distanceR());
        //delay - no point in printing readings more  frequently than the user can read
        //note that the sonar will continue to be taking measurements continually
        //but will only send these values to Arduino when you run update() function        
        delay(300);
    }
}


/*
 * 
 *     Let the user enter commands at serial monitor
 *    Available commands (each is one char)
 *      r : restart sensor
 *      s : stop sensor
 */
 
char readChar() {
    char c=' ';
    if (Serial.available()) {
        c=Serial.read();
    }
    return c;
}



