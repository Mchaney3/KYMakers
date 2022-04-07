// Combine this with software serial and other code from device example in TINYGPS++ ESP32 Examples

#include <TinyGPS++.h>

static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

// Compass navigation
#include <Wire.h>
#include <HMC5883L.h>
HMC5883L compass;
int targetHeading;              // arah sudut yang dituju
int currentHeading;             // arah sudut saat ini
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 8     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

#include "math.h"
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Serial2);
float currentLat,
      currentLong;

int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it

#define WAYPOINT_DIST_TOLERANE  3   //tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define stopwaypoint 5                  //second
int current_waypoint = 0;
// GESER KESELATAN -lat
float waypointList[10][2] = {
  { -7.297828, 112.801938},
  { -7.297885, 112.802251}, //-7.297873, 112.802250
  { -7.297931, 112.802584}, //-7.297925, 112.802584 //-7.298058, 112.803043
};

float targetLat = waypointList[0][0];
float targetLong = waypointList[0][1];
void setup()
{
  Serial.begin(9600);         //Debug
  //Serial3.begin(9600);       //Bluetooth
  Serial2.begin(9600);        //GPS

  pinMode(19, INPUT_PULLUP);
  setup_motor();


  GPS.begin(9600);                                // 9600 NMEA default speed
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);     // turns on RMC and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);       // 1 Hz update rate
  GPS.sendCommand(PGCMD_NOANTENNA);                // turn off antenna status info
  delay(1000);


  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
  Serial.println("Start");

  Serial.println(waypointList[0][0], 6);
  Serial.println(waypointList[0][1], 6);

  Serial.println(waypointList[1][0], 6);
  Serial.println(waypointList[1][1], 6);

}

String inputString = "";
String inputdata = "";
bool stringComplete = false;
int id = 0;
int mode;

String val[5];
int nprint = 0;
int aprint = 0;

void loop()
{
  int sensorVal = digitalRead(19);

  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      processGPS();

  /////////////////////////Data dari hp
  if (stringComplete)
  {
    Serial.println(inputString);
    int n = 0;
    for (int i = 0; i < 30; i++)
    {
      if (inputString[i] == 0)
      {
        n++;
        if (n == 4)
          break;
      }
      else
      {
        val[n] += inputString[i];
      }
    }

    if (val[1] == "1")
    {
      targetLat = val[2].toFloat();
      targetLong = val[3].toFloat();
    }

    /////////////////////////////////// Toogle Switch
    inputString = "";
    val[1] = "";
    val[2] = "";
    val[3] = "";
    stringComplete = false;
  }


  if (sensorVal == HIGH)
  {
    berhenti();
    //    Serial.print("la "); Serial.print(currentLat, 6);
    //    Serial.print("_lo "); Serial.print(currentLong, 6);
    //    Serial.print("_la "); Serial.print(targetLat, 6);
    //    Serial.print("_lo "); Serial.print(targetLong, 6);
    //    Serial.println();
    nprint++;
    if (nprint == 10000)
    {
      nprint = 0;

     // Serial3.print("la "); Serial3.print(currentLat, 6);
      //Serial3.print("_lo "); Serial3.print(currentLong, 6);
      //Serial3.print("_la "); Serial3.print(targetLat, 6);
      //Serial3.print("_lo "); Serial3.print(targetLong, 6);
      //Serial3.println();
    }
  }
  else if (sensorVal == LOW)
  {
    // memastikan ada data GPS di robot dan HP
    if (currentLong > 1 && targetLong > 1)
    {
      currentHeading = readCompass();
      calcDesiredTurn();
      move_robot();

      //      Serial.print("_ch "); Serial.print(currentHeading);
      //      Serial.print("_th "); Serial.print(targetHeading);
      //      Serial.print("_eh "); Serial.print(headingError);
      //      Serial.print("_di "); Serial.print(distanceToTarget);
      //      Serial.println();


      aprint++;
      if (aprint == 200 )
      {
        aprint = 0;
        //Serial3.print("wp "); Serial3.print(current_waypoint);
        //Serial3.print("_ch "); Serial3.print(currentHeading);
        //Serial3.print("_th "); Serial3.print(targetHeading);
        //Serial3.print("_eh "); Serial3.print(headingError);
        //Serial3.print("_di "); Serial3.print(distanceToTarget);
        //Serial3.println();
      }

    }
    else
    {
      berhenti();
    }
  }
}

void processGPS()
{
  if (gps.location.isValid())
  {
    currentLat = gps.location.lat();
    currentLong = gps.location.lng();

    // update the course and distance to waypoint based on our new position
    distanceToWaypoint();
    courseToWaypoint();
  }
}

// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// copied from TinyGPS library
int courseToWaypoint()
{
  float dlon = radians(targetLong - currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
  return targetHeading;
}

// returns distance in meters between two positions, both specified
// as signed decimal-degrees latitude and longitude. Uses great-circle
// distance computation for hypothetical sphere of radius 6372795 meters.
// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
// copied from TinyGPS library
int distanceToWaypoint()
{
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  distanceToTarget =  delta * 6372795;

  // check to see if we have reached the current waypoint
  if (distanceToTarget <= WAYPOINT_DIST_TOLERANE-1)
  {
    berhenti();
    delay(stopwaypoint*1000);
    nextWaypoint();
  }
  return distanceToTarget;
}


int readCompass()
{
  Vector norm = compass.readNormalize();
  float heading = atan2(norm.YAxis, norm.XAxis);  // Calculate heading

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  //float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  // For Surabaya declination angle is 0'54E (positive)
  float declinationAngle = (0 + (54.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)  heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;

  // Convert to degrees
  float headingDegrees = heading * 180 / M_PI;
  return ((int)headingDegrees);
}

void nextWaypoint()
{
  current_waypoint++;
  targetLat = waypointList[current_waypoint][0];
  targetLong = waypointList[current_waypoint][1];

  if ((targetLat == 0 && targetLong == 0))    // last waypoint reached?
  {
    berhenti();
    //Serial3.println("FINISH");
    while (1); //PRESS RESET BUTTON
  }
  else
  {
    processGPS();
    distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
    courseToWaypoint();
  }
}


int error_output;
void calcDesiredTurn(void)
{
  headingError = targetHeading - currentHeading;  // calculate where we need to turn to head to destination

  // adjust for compass wrap
  if (headingError < -180)
    headingError  += 360;
  if (headingError > 180)
    headingError -= 360;


  // calculate which way to turn to intercept the targetHeading
  if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
    error_output = 0;
  else if (headingError < 60 && headingError > -60)
  {
    error_output = headingError;
  }
  else if (headingError >= 60)
    error_output = 100;
  else if (headingError <= -60)
    error_output = -100;

}
float Kp, Kd;
void move_robot()
{
  if (error_output == 0)
  {
    majuGo(60, 60); //0-255
  }
  else if (error_output < 60)
  {
    if (error_output < 0)
      majuGo(60 + error_output, 60);
    else
      majuGo(60, 60 - error_output);
  }
  else if (error_output == 100)
  {
    majuGo(50, -50); //0-255
  }
  else if (error_output == -100)
  {
    majuGo(50, -50); //0-255
  }
}

/*
void serialEvent3() //bluetooth
{
  while (Serial3.available())
  {
    char inChar = (char)Serial3.read();
    inputString += inChar;
    if (inChar == 'v')
    {
      stringComplete = true;
    }
  }
}
*/
