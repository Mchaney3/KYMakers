# Drive

## L298N Motor Driver
Forward, Left, Right and Back control

## RC Receiver
Receiving signal and converted to PWM signal by ESP32

## AutoPilot
PixHawk 4 to start with. This is the expensive option. Final version should run with DroneBridge or some other open source mission planning software running on low cost IoT hardware.


# Cut

## Motor(s)
### Need to get Shaft diameter for blade
Will use 2200kv brushless and 30 amp ESC for v1. 

## Blade(s)
Blades to be fabricated based on motor specifications

## SAFETY
Safety first, that's why it's item # 3 for Milestone # 2


# Object Detection

## Ultrasonic Sensor
Mounted on servo for 360 degree object detection

## Servo
360 degree continuous servo


# Tracking

## SD Card
Data logging and diagnostics

## HMC5883 (Compass)
Calculate heading at startup

## GPS
Real world location tracking, auto pilot, waypoint missions


# Interface

## Buzzer
Provide audio feedback

## SSD1306 (128x32 OLED)
Provide Visual feedback for diagnostics in v1. v2 and later can utilize for battery charge and other features.

## Web
Not sure if this is necessary

## Bluetooth
Not sure if necessary but an option

## LED
Feedback for diagnostics + looks super cool


# Cloud

## Server Infrastructure
Determine best cloud solution. Right now, Blynkk Local Server looks most promising. Other considerations are RainMaker and Home Assistant

## OTA
Goal of cloud solution is both real time onformaiton for user but also real time management for developer
