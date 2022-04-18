/***************************************************************************
  
  Prints GPS  readings to serial monitor (you need to open serial monitor at 9600 Baud)
 ***************************************************************************/

#include <SoftwareSerial.h>
#include <TinyGPS++.h>
static const int RXPin = 5, // the serial connection to GPS. Note that RX, TX
                 TXPin = 4, // refer to 'device view'
                 SDAPin = 0, SCLPin = 2,//I2C pins
                 Motor1aPin = 12, Motor1bPin = 14,
                 Motor2aPin = 13, Motor2bPin = 15;


// GPS sensor BAUD rate
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;
// Setup serial connection to the GPS module
SoftwareSerial ss(TXPin, RXPin, 0, 512); //increase buffer size to 512


void setup(void)  {
    //start serial monitor
    Serial.begin(9600);
    ss.begin(GPSBaud);
    Serial.println("Compass Test"); Serial.println("");
}

void loop(void)  {
    delay(500);
    updateGPS();
    if ( !gps.location.isValid()) {
        //no location fix yet;
         Serial.println("No location fix...");
    } else if (  gps.location.age()>3000   ) {
        //fix is more than 3 seconds old - we lost signal
         Serial.println("Signal lost");
    } else {
        // everything is OK - signal is valid and not old 
        Serial.println();
        Serial.print("Location: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(", ");
        Serial.print(gps.location.lng(), 6);                
   }
}

void updateGPS() {
    //read data from serial connection to GPS
    while (ss.available() > 0) {
        gps.encode(ss.read());
    }
}
