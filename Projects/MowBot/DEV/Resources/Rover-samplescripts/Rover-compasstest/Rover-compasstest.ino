/***************************************************************************
  Based on an example  by Kevin Townsend written for Adafruit Industries, 
  Prints magnetometer (compass) readings to serial monitor (you need to open serial monitor at 9600 Baud)
 ***************************************************************************/
//I2C library 
#include <Wire.h>
//Adafruit sensor libraries. See https://github.com/adafruit/Adafruit_HMC5883_Unified
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

static const int RXPin = 5, // the serial connection to GPS. Note that RX, TX
                 TXPin = 4, // refer to 'device view'
                 SDAPin = 0, SCLPin = 2,//I2C pins
                 Motor1aPin = 12, Motor1bPin = 14,
                 Motor2aPin = 13, Motor2bPin = 15;


/* Create new magnetometer (compass)  sensor, with random ID */
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);


void setup(void)  {
    //start serial monitor
    Serial.begin(9600);
    Serial.println("Compass Test"); Serial.println("");
    Wire.begin(SDAPin, SCLPin);
    // Initialise the sensor 
    if (!compass.begin()) {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        while(1);//just stop forever
    }   
}

void loop(void)  {
    // Get a new sensor reading (event) 
    sensors_event_t event; 
    compass.getEvent(&event);
    
    float X,Y;
    // Get x,y components of magnetic field vector; values are in micro-Tesla (uT)) 
    X=event.magnetic.x;
    Y=event.magnetic.y;
    /* 
     * correct for bias. For reasons unknown, the compass sensors we use have some bias:
     * the value they return differs from true value by 
     * x_returned=x_true +x_offset
     * thus, to get correct value, we must subtract offset
     * To find bias, rotate the sensor 360 degrees computing the maximal and minimal value; then 
     * x_offset=(x_returned_min +x_returned_max)/2
     */
    float Xoffset=-10, Yoffset=-10 ; 
    X-=Xoffset;
    Y-=Yoffset;
    Serial.print("X: "); Serial.print(X); Serial.print("  ");
    Serial.print("Y: "); Serial.print(Y); Serial.print("  ");
    Serial.println("uT");
    
    // Calculate heading  and convert radians to degrees
    // Heading = angle from north, counted clockwise (so east would be 90, west=270) 
    float heading = atan2(-X, -Y) * 180/PI;
    
    /* Once you have your heading, you must then add your 'Declination Angle', 
     * to account for difference between magnetic north and true north which is the 'Error' of the magnetic field in your location.
     * See here: http://www.magnetic-declination.com/
     * For New York, declination is -13.2 (west); that is, magnetic north is 13 degrees west of true north. 
     */
    float declinationAngle =-13.2;
    heading += declinationAngle;
    
    // correct as needed to guarantee that result is between 0 and 360
    if (heading < 0) heading += 360;
    if (heading > 360 ) heading -= 360;
    Serial.print("Heading (degrees): "); Serial.println(heading);
    
    delay(500);
}
