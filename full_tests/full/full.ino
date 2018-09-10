#include <Wire.h>
#include<SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include "compass.h"
#include "TinyGPS.h"

LSM303 compass;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
}
float t = 0;
void loop() {
  // put your main code here, to run repeatedly:
  compass.read();
  t = compass.heading();//Compass in degrees.
  
}
