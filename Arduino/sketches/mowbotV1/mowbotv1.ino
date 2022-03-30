
// Include Libraries
#include "Arduino.h"
#include "Servo.h"
#include "DCMDriverL298.h"
#include "DHT.h"
#include "NewPing.h"
#include "HMC5883L.h"
#include "Wire.h"
#include "RTClib.h"
#include "SD.h"


// Pin Definitions
#define BRUSHLESSMOTOR_PIN_DATA	0
#define DCMOTORDRIVERL298_PIN_INT1	6
#define DCMOTORDRIVERL298_PIN_ENB	4
#define DCMOTORDRIVERL298_PIN_INT2	7
#define DCMOTORDRIVERL298_PIN_ENA	2
#define DCMOTORDRIVERL298_PIN_INT3	8
#define DCMOTORDRIVERL298_PIN_INT4	9
#define DHT_PIN_DATA	10
#define HCSR04_PIN_TRIG	13
#define HCSR04_PIN_ECHO	11
#define IRPROXIMITY_PIN_VOUT	12
#define LEDWS2812BREAKOUT_PIN_DI	14
#define SDFILE_PIN_CS	5
#define SERVO360_PIN_SIG	15



// Global variables and defines
const int brushlessMotorLowSpeed = 1000;  //Starting speed
const int brushlessMotorFastSpeed = 2000; //Top speed
// object initialization
Servo brushlessMotor;
DCMDriverL298 dcMotorDriverL298(DCMOTORDRIVERL298_PIN_ENA,DCMOTORDRIVERL298_PIN_INT1,DCMOTORDRIVERL298_PIN_INT2,DCMOTORDRIVERL298_PIN_ENB,DCMOTORDRIVERL298_PIN_INT3,DCMOTORDRIVERL298_PIN_INT4);
DHT dht(DHT_PIN_DATA);
NewPing hcsr04(HCSR04_PIN_TRIG,HCSR04_PIN_ECHO);
HMC5883L compass;
RTC_PCF8523 rtcPCF;
File sdFile;
Servo servo360;


// define vars for testing menu
const int timeout = 10000;       //define timeout of 10 sec
char menuOption = 0;
long time0;

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() 
{
    // Setup Serial which is useful for debugging
    // Use the Serial Monitor to view printed messages
    Serial.begin(9600);
    while (!Serial) ; // wait for serial port to connect. Needed for native USB
    Serial.println("start");
    
    // WARNING! DO NOT CONNECT THE PROPELLER UNDER TEST!
    brushlessMotor.attach(BRUSHLESSMOTOR_PIN_DATA);
    brushlessMotor.writeMicroseconds(brushlessMotorLowSpeed);
    dht.begin();
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For declination angle 4'36E (positive)
    compass.begin(+4,36);
    if (! rtcPCF.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
    }
    if (! rtcPCF.initialized()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtcPCF.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtcPCF.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
    // Set SPI SS pin to output otherwise the SD library functions will not work.
    // The SD is set to use SPI SS Arduino pin 10 as chip select(CS) by default.
    // To change the pin use SD.begin(SD_CS_PIN)
    pinMode(SDFILE_PIN_CS, OUTPUT);
    // Check if the card is present and can be initialized
    if (!SD.begin()) {
    Serial.println(F("Card failed, or not present"));
    while(1);
    }
    Serial.println(F("card initialized."));
    menuOption = menu();
    
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{
    
    
    if(menuOption == '1') {
    // A2212 Brushless Motor 1000KV (with 30A ESC) - Test Code
    // The motor will rotate, statrting from low speed to high speed gradually.
    // WARNING! DO NOT CONNECT THE PROPELLER UNDER TEST!
    // In order to reverse the motor direction disconnect the power from the circuit and replace any 2 of the 3 blue wires between the motor and the ESC (marked: 30A OPTO)
    for(int brushlessMotorRotationSpeed = brushlessMotorLowSpeed; brushlessMotorRotationSpeed <=  brushlessMotorFastSpeed; brushlessMotorRotationSpeed += 10)
    {
    brushlessMotor.writeMicroseconds(brushlessMotorRotationSpeed);  // 2. spin the motor at the rated speed. This number gradually changes during the 'for' loop
    Serial.println(brushlessMotorRotationSpeed);
    delay(100);                              // 3. waits 100 milliseconds (0.1 sec). change the value in the brackets (100) for a longer or shorter delay in milliseconds.
    }
    for(int brushlessMotorRotationSpeed = brushlessMotorFastSpeed; brushlessMotorRotationSpeed >=  brushlessMotorLowSpeed; brushlessMotorRotationSpeed -= 10)
    {
    brushlessMotor.writeMicroseconds(brushlessMotorRotationSpeed);  // 2. spin the motor at the rated speed. This number gradually changes during the 'for' loop
    Serial.println(brushlessMotorRotationSpeed);
    delay(100);                              // 3. waits 100 milliseconds (0.1 sec). change the value in the brackets (100) for a longer or shorter delay in milliseconds.
    }
    brushlessMotor.detach();                    // 4. release the brushless motor to conserve power and accidental operation.
    }
    else if(menuOption == '2')
    {
    // Disclaimer: The INA219 High Side DC Current Sensor Breakout - 26V 3.2A Max is in testing and/or doesn't have code, therefore it may be buggy. Please be kind and report any bugs you may find.
    }
    else if(menuOption == '3') {
    // L298N Motor Driver with Dual Micro DC Motors (Geared) - Test Code
    //Start both motors. note that rotation direction is determined by the motors connection to the driver.
    //You can change the speed by setting a value between 0-255, and set the direction by changing between 1 and 0.
    dcMotorDriverL298.setMotorA(200,1);
    dcMotorDriverL298.setMotorB(200,0);
    delay(2000);
    //Stop both motors
    dcMotorDriverL298.stopMotors();
    delay(2000);

    }
    else if(menuOption == '4') {
    // DHT22/11 Humidity and Temperature Sensor - Test Code
    // Reading humidity in %
    float dhtHumidity = dht.readHumidity();
    // Read temperature in Celsius, for Fahrenheit use .readTempF()
    float dhtTempC = dht.readTempC();
    Serial.print(F("Humidity: ")); Serial.print(dhtHumidity); Serial.print(F(" [%]\t"));
    Serial.print(F("Temp: ")); Serial.print(dhtTempC); Serial.println(F(" [C]"));

    }
    else if(menuOption == '5')
    {
    // Disclaimer: The Ublox NEO-6M GPS Module is in testing and/or doesn't have code, therefore it may be buggy. Please be kind and report any bugs you may find.
    }
    else if(menuOption == '6') {
    // Ultrasonic Sensor - HC-SR04 - Test Code
    // Read distance measurment from UltraSonic sensor           
    int hcsr04Dist = hcsr04.ping_cm();
    delay(10);
    Serial.print(F("Distance: ")); Serial.print(hcsr04Dist); Serial.println(F("[cm]"));

    }
    else if(menuOption == '7') {
    // HMC5883L - Magnetometer (Compass) - Test Code
    //Read heading value from compass
    float compassHeading = compass.getHeadingDeg();
    Serial.print(F("Heading: ")); Serial.print(compassHeading); Serial.println(F("[°]"));

    }
    else if(menuOption == '8')
    {
    // Disclaimer: The Infrared Proximity Sensor Long Range - Sharp GP2Y0A02YK0F is in testing and/or doesn't have code, therefore it may be buggy. Please be kind and report any bugs you may find.
    }
    else if(menuOption == '9')
    {
    // Disclaimer: The SparkFun RGB LED Breakout - WS2812B is in testing and/or doesn't have code, therefore it may be buggy. Please be kind and report any bugs you may find.
    }
    else if(menuOption == '10') {
    // Adafruit PCF8523 Real Time Clock Assembled Breakout Board - Test Code
    //This will display the time and date of the RTC. see RTC.h for more functions such as rtcPCF.hour(), rtcPCF.month() etc.
    DateTime now = rtcPCF.now();
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print('/');
    Serial.print(now.year(), DEC);
    Serial.print("  ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    delay(1000);
    }
    else if(menuOption == '11') {
    // Micro SD module - Test Code
    // The SD example code creates a datalog.txt file for logging sensor data
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    sdFile = SD.open("datalog.txt", FILE_WRITE);
    // if the file exists in SD card, write sensor data
    if (sdFile) {
    //Write to file
    sdFile.println("ENTER SENSOR DATA HERE");
    // close the file
    sdFile.close();
    // Uncomment to print to the serial port
    //Serial.println("ENTER SENSOR DATA HERE");
    } 
    else {
    // if the file didn't open, print an error
    Serial.println(F("error opening file."));
    }
    }
    else if(menuOption == '12') {
    // Servo - Generic Continuous Rotation (Micro Size) - Test Code
    // The servo will rotate CW in full speed, CCW in full speed, and will stop  with an interval of 2000 milliseconds (2 seconds) 
    servo360.attach(SERVO360_PIN_SIG);         // 1. attach the servo to correct pin to control it.
    servo360.write(180);  // 2. turns servo CW in full speed. change the value in the brackets (180) to change the speed. As these numbers move closer to 90, the servo will move slower in that direction.
    delay(2000);                              // 3. waits 2000 milliseconds (2 sec). change the value in the brackets (2000) for a longer or shorter delay in milliseconds.
    servo360.write(0);    // 4. turns servo CCW in full speed. change the value in the brackets (0) to change the speed. As these numbers move closer to 90, the servo will move slower in that direction.
    delay(2000);                              // 5. waits 2000 milliseconds (2 sec). change the value in the brackets (2000) for a longer or shorter delay in milliseconds.
    servo360.write(90);    // 6. sending 90 stops the servo 
    delay(2000);                              // 7. waits 2000 milliseconds (2 sec). change the value in the brackets (2000) for a longer or shorter delay in milliseconds.
    servo360.detach();                    // 8. release the servo to conserve power. When detached the servo will NOT hold it's position under stress.
    }
    
    if (millis() - time0 > timeout)
    {
        menuOption = menu();
    }
    
}



// Menu function for selecting the components to be tested
// Follow serial monitor for instrcutions
char menu()
{

    Serial.println(F("\nWhich component would you like to test?"));
    Serial.println(F("(1) A2212 Brushless Motor 1000KV (with 30A ESC)"));
    Serial.println(F("(2) INA219 High Side DC Current Sensor Breakout - 26V 3.2A Max"));
    Serial.println(F("(3) L298N Motor Driver with Dual Micro DC Motors (Geared)"));
    Serial.println(F("(4) DHT22/11 Humidity and Temperature Sensor"));
    Serial.println(F("(5) Ublox NEO-6M GPS Module"));
    Serial.println(F("(6) Ultrasonic Sensor - HC-SR04"));
    Serial.println(F("(7) HMC5883L - Magnetometer (Compass)"));
    Serial.println(F("(8) Infrared Proximity Sensor Long Range - Sharp GP2Y0A02YK0F"));
    Serial.println(F("(9) SparkFun RGB LED Breakout - WS2812B"));
    Serial.println(F("(10) Adafruit PCF8523 Real Time Clock Assembled Breakout Board"));
    Serial.println(F("(11) Micro SD module"));
    Serial.println(F("(12) Servo - Generic Continuous Rotation (Micro Size)"));
    Serial.println(F("(menu) send anything else or press on board reset button\n"));
    while (!Serial.available());

    // Read data from serial monitor if received
    while (Serial.available()) 
    {
        char c = Serial.read();
        if (isAlphaNumeric(c)) 
        {   
            
            if(c == '1') 
    			Serial.println(F("Now Testing A2212 Brushless Motor 1000KV (with 30A ESC)"));
    		else if(c == '2') 
    			Serial.println(F("Now Testing INA219 High Side DC Current Sensor Breakout - 26V 3.2A Max - note that this component doesn't have a test code"));
    		else if(c == '3') 
    			Serial.println(F("Now Testing L298N Motor Driver with Dual Micro DC Motors (Geared)"));
    		else if(c == '4') 
    			Serial.println(F("Now Testing DHT22/11 Humidity and Temperature Sensor"));
    		else if(c == '5') 
    			Serial.println(F("Now Testing Ublox NEO-6M GPS Module - note that this component doesn't have a test code"));
    		else if(c == '6') 
    			Serial.println(F("Now Testing Ultrasonic Sensor - HC-SR04"));
    		else if(c == '7') 
    			Serial.println(F("Now Testing HMC5883L - Magnetometer (Compass)"));
    		else if(c == '8') 
    			Serial.println(F("Now Testing Infrared Proximity Sensor Long Range - Sharp GP2Y0A02YK0F - note that this component doesn't have a test code"));
    		else if(c == '9') 
    			Serial.println(F("Now Testing SparkFun RGB LED Breakout - WS2812B - note that this component doesn't have a test code"));
    		else if(c == '10') 
    			Serial.println(F("Now Testing Adafruit PCF8523 Real Time Clock Assembled Breakout Board"));
    		else if(c == '11') 
    			Serial.println(F("Now Testing Micro SD module"));
    		else if(c == '12') 
    			Serial.println(F("Now Testing Servo - Generic Continuous Rotation (Micro Size)"));
            else
            {
                Serial.println(F("illegal input!"));
                return 0;
            }
            time0 = millis();
            return c;
        }
    }
}

/*******************************************************

*    Circuito.io is an automatic generator of schematics and code for off
*    the shelf hardware combinations.

*    Copyright (C) 2016 Roboplan Technologies Ltd.

*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.

*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.

*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*    In addition, and without limitation, to the disclaimers of warranties 
*    stated above and in the GNU General Public License version 3 (or any 
*    later version), Roboplan Technologies Ltd. ("Roboplan") offers this 
*    program subject to the following warranty disclaimers and by using 
*    this program you acknowledge and agree to the following:
*    THIS PROGRAM IS PROVIDED ON AN "AS IS" AND "AS AVAILABLE" BASIS, AND 
*    WITHOUT WARRANTIES OF ANY KIND EITHER EXPRESS OR IMPLIED.  ROBOPLAN 
*    HEREBY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT 
*    NOT LIMITED TO IMPLIED WARRANTIES OF MERCHANTABILITY, TITLE, FITNESS 
*    FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, AND THOSE ARISING BY 
*    STATUTE OR FROM A COURSE OF DEALING OR USAGE OF TRADE. 
*    YOUR RELIANCE ON, OR USE OF THIS PROGRAM IS AT YOUR SOLE RISK.
*    ROBOPLAN DOES NOT GUARANTEE THAT THE PROGRAM WILL BE FREE OF, OR NOT 
*    SUSCEPTIBLE TO, BUGS, SECURITY BREACHES, OR VIRUSES. ROBOPLAN DOES 
*    NOT WARRANT THAT YOUR USE OF THE PROGRAM, INCLUDING PURSUANT TO 
*    SCHEMATICS, INSTRUCTIONS OR RECOMMENDATIONS OF ROBOPLAN, WILL BE SAFE 
*    FOR PERSONAL USE OR FOR PRODUCTION OR COMMERCIAL USE, WILL NOT 
*    VIOLATE ANY THIRD PARTY RIGHTS, WILL PROVIDE THE INTENDED OR DESIRED
*    RESULTS, OR OPERATE AS YOU INTENDED OR AS MAY BE INDICATED BY ROBOPLAN. 
*    YOU HEREBY WAIVE, AGREE NOT TO ASSERT AGAINST, AND RELEASE ROBOPLAN, 
*    ITS LICENSORS AND AFFILIATES FROM, ANY CLAIMS IN CONNECTION WITH ANY OF 
*    THE ABOVE. 
********************************************************/