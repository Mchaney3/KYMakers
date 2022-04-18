
static const int RXPin = 5, // the serial connection to GPS. Note that RX, TX
                 TXPin = 4, // refer to 'device view'
                 SDAPin = 0, SCLPin = 2,//I2C pins
                 Motor1aPin = 12, Motor1bPin = 14,
                 Motor2aPin = 13, Motor2bPin = 15;
void setup() {
    //set the motor control pins
    pinMode(Motor1aPin, OUTPUT);
    pinMode(Motor1bPin, OUTPUT);
    pinMode(Motor2aPin, OUTPUT);
    pinMode(Motor2bPin, OUTPUT);
}

void loop(){
    setMotors(1,1); delay(1000);
    setMotors(1,0); delay(700);
    setMotors(0,1); delay(700);
    setMotors(0,0); delay(1000);
    setMotors(-1,-1);delay(1000);
    setMotors(0,0); delay(3000);
}
/*
 *  Set motors. Each motor power should be float between -1 and 1
 *  If values are outside of this range, both values will be rescaled:  
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
    // unlike the usual Arduino, which expects range of 0-255
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


