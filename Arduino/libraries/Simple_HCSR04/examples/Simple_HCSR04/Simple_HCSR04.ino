/**
 * @file Simple_HCSR04.ino
 * @author Moritz Bergmann
 * @brief An example for unsing the Simple_HCSR04 library.
 * @version 1.0
 * @date 2020-12-18
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <Arduino.h>
#include <Simple_HCSR04.h>

const int ECHO_PIN = 4; /// the pin at which the sensor echo is connected
const int TRIG_PIN = 5; /// the pin at which the sensor trig is connected

Simple_HCSR04 *sensor;

void setup()
{
  Serial.begin(9600);

  // create sensor object
  sensor = new Simple_HCSR04(ECHO_PIN, TRIG_PIN);
}

void loop()
{
  unsigned long distance = sensor->measure()->cm();

  Serial.print("distance: ");
  Serial.print(distance);
  Serial.print("cm\n");
}
