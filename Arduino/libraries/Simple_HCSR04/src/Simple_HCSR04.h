/**
 * @file Simple_HCSR04.h
 * @author Moritz Bergmann
 * @brief The header file for the wrapper.
 * @version 1.0
 * @date 2020-12-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef HCSR04_H_
#define HCSR04_H_

#include <Arduino.h>

/**
 * @brief A simple wrapper for measuring the 
 * distance with an HC-SR04 Module.
 *
 */
class Simple_HCSR04
{
private:

    /**
     * The Object should only be initialized with members,
     * therefore the default constructor is private.
     * 
     */
    Simple_HCSR04();

    /// The echo pin.
    const short ECHO_PIN;
    /// The trig pin.
    const short TRIG_PIN;

public:
    /**
     * @brief A simple wrapper for holding the measured distance.
     *
     */
    class Measurement
    {
    private:
        /**
         * The Object should only be initialized with members,
         * therefore the default constructor is private.
         * 
         */
        Measurement();

        const unsigned long m_measurement; /// The measured distance.

    public:
        /**
         * @brief Construct a new Simple_HCSR04::Measurement::Measurement object from a given number.
         * The given number should be provided in centimeters to provide a reasonable conversion.
         *
         * @param measurement distance
         */
        Measurement(unsigned long measurement);

        /**
         * @brief Measurement/ Distance in millimeters.
         *
         * @return unsigned long
         */
        unsigned long mm() const;

        /**
         * @brief Measurement/ Distance in centimeters.
         *
         * @return unsigned long
         */
        unsigned long cm() const;

        /**
         * @brief Measurement/ Distance in decimeters.
         *
         * @return float
         */
        float dm() const;

        /**
         * @brief Measurement/ Distance in meters.
         *
         * @return float
         */
        float m() const;

    };

    /**
     * @brief Construct a new Simple_HCSR04::Simple_HCSR04 object and
     * initializes the pin mode for the given pins.
     *
     * @param echo_pin Echo pin on the microcontroller (warning, is being cast to an unsigned short).
     * @param trig_pin Trig pin on the microcontroller (warning, is being cast to an unsigned short).
     */
    Simple_HCSR04(const short echo_pin, const short trig_pin);

    /**
     * @brief Getter for the Echo pin.
     *
     * @return short
     */
    short getEchoPin() const;

    /**
     * @brief Getter for the Trig pin.
     *
     * @return short
     */
    short getTrigPin() const;

    /**
     * @brief Measures the distance between the module and the object in front of it.
     * Warning: this function needs at least 12 milliseconds to execute.
     *
     * @return Simple_HCSR04::Measurement*
     */
    Measurement* measure();

};

#endif
