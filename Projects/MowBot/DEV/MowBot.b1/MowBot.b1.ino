#include <ESP32Servo.h>
#include <analogWrite.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>

#include <IBusBM.h>

IBusBM IBus; // IBus object for receivig signals from transmitter/receiver

void setup() {
  Serial.begin(115200);   

  IBus.begin(Serial2);    // iBUS connected to Serial1: 19 (RX) and 18 (TX)

  // We have to wait for the receiver to receive data from the transmitter (transmitter needs to be turned on)
  // as the channel values all read 0 as long as the transmitter is turned off at boot time.
  // We do not want the car to drive full speed backwards out of control.
  Serial.println("Wait for receiver");
  while (IBus.cnt_rec==0) delay(100);
  Serial.println("Init done");
}

// braking not used in thix example
void brake(int brakePower) {
//  Motor1.brake(brakePower);
//  Motor2.brake(brakePower);
}

void speedturn(int speed, int angle) {
  // set speed (-400 -> +400) and turn (-400 -> +400)
  // turn vehicle by providing different speed settings to the motors.
  // angle can be positive (right turn) or negative (left turn).
  // If the vehicle is already stopped, the vehicle will turn in place.
  // Motor1.setSpeed(speed + angle);
  // Motor2.setSpeed(speed - angle);
}

int savespd=0, saveturn=0;

void loop() {
  int spd, turn;
  // speed depends on front switch (channel 5) (forward/backwards) and channel 2 (speed)
  spd = ((int) IBus.readChannel(2)-1050); 
  // every value below 1050 we interprete as stop 
  if (spd<0) spd=0; else spd = (spd*4)/9; // value could reach (2000-1050)*4/9 = 422, but setspeed() will max at 400
  if (IBus.readChannel(5)>1500) spd=-spd; // backward/forward depends on switch at channel 5
  
  // turn depends on channel 0, scale down to -200, +200
  turn = (((int) IBus.readChannel(0)-1500)*4)/10; 

  // set combined speed and turn (if speed==0, then only turn in place)
  speedturn(spd, turn);

  if (savespd != spd || saveturn != turn) {
    Serial.print("speed="); Serial.print(spd); // display speed
    Serial.print(" turn="); Serial.println(turn); // display turn 
    savespd = spd;
    saveturn = turn;
  }
  delay(100);
}
