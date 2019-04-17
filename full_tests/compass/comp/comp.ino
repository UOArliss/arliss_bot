#include <Wire.h>
#include "compass.h"

LSM303 compass;
//LSM303::vector<int16_t> running_min = {32229, 32767, 32767}, running_max = {-32059, -32768, -32768};

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
}
float t=0;
void loop() {  
  compass.read();

  
  
  //Serial.println("Hi");
  t = compass.heading();
  Serial.print("Current approximate heading: ");
  Serial.println(t);;//Serial.print(compass.debug.x);Serial.print(' ');Serial.print(compass.debug.y);Serial.print(' ');Serial.println(compass.debug.z);;
  delay(100);
}

template <typename Ta, typename Tb> void LSM303::calibrate_offset(vector<Ta>* mmin , vector<Tb>* mmax){

  //Does nothing.
}
