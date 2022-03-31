/**
 * @file Simple_HCSR04.cpp
 * @author Moritz Bergmann
 * @brief The cpp file for the wrapper.
 * @version 1.0
 * @date 2020-12-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <Arduino.h>
#include "Simple_HCSR04.h"

/***
 * Measurement
 */

Simple_HCSR04::Measurement::Measurement(unsigned long measurement)
    : m_measurement(measurement) {}


/***
 * Measurement getter
 */

unsigned long Simple_HCSR04::Measurement::mm() const
{
    return this->m_measurement * 10;
}

unsigned long Simple_HCSR04::Measurement::cm() const
{
    return this->m_measurement;
}

float Simple_HCSR04::Measurement::dm() const
{
    return this->m_measurement / 10;
}

float Simple_HCSR04::Measurement::m() const
{
    return this->m_measurement / 100;
}



/***
 * Simple_HCSR04
 */

Simple_HCSR04::Simple_HCSR04(const short echo_pin, const short trig_pin)
    : ECHO_PIN(echo_pin), TRIG_PIN(trig_pin)
{
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
}


/***
 * Simple_HCSR04 getter
 */

short Simple_HCSR04::getEchoPin() const
{
    return this->ECHO_PIN;
}

short Simple_HCSR04::getTrigPin() const
{
    return this->TRIG_PIN;
}


/***
 * Simple_HCSR04 methods
 */

Simple_HCSR04::Measurement* Simple_HCSR04::measure()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH);
    unsigned long distance = duration * 0.034 / 2;

    return new Simple_HCSR04::Measurement(distance);
}
