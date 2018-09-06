#include <Wire.h>
#include "compass.h"

LSM303 compass;
LSM303::vector<int16_t> running_min = {32229, 32767, 32767}, running_max = {-32059, -32768, -32768};

char report[110];

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
}
float x=0,y=0,z=0;
void loop() {  
  compass.read();
  //min: {  -290,    -17,  -1689}    max: {  +820,  +1121,   -508}

  running_min.x = min(running_min.x, compass.m.x);
  running_min.y = min(running_min.y, compass.m.y);
  running_min.z = min(running_min.z, compass.m.z);

  running_max.x = max(running_max.x, compass.m.x);
  running_max.y = max(running_max.y, compass.m.y);
  running_max.z = max(running_max.z, compass.m.z);
  
  snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}  val: {%+6d, %+6d, %+6d}",
    running_min.x, running_min.y, running_min.z,
    running_max.x, running_max.y, running_max.z,
    compass.m.x, compass.m.y, compass.m.z);
  Serial.println(report);
  
  //Serial.println("Hi");
  x = (((float)(compass.m.x - running_min.x))/((float)(running_max.x - running_min.x)))*(200.0) - 100.0;
  y = (((float)(compass.m.y - running_min.y))/((float)(running_max.y - running_min.y)))*(200.0) - 100.0;
  z = (((float)(compass.m.z - running_min.z))/((float)(running_max.z - running_min.z)))*(200.0) - 100.0;
  //high
  //need North X and East X, atan2 the two values....
  //cross acc and mag for east,
  //cross east and acc for north
  
  Serial.print(x); Serial.print(' '); Serial.print(y); Serial.print(' ');Serial.println(z);
  delay(100);
}

template <typename Ta, typename Tb> void LSM303::calibrate_offset(vector<Ta>* mmin , vector<Tb>* mmax){

  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

  uint32_t time = 60000L; 

  for( uint32_t start = millis(); (millis()-start) < time;){
    compass.read();
    mmin->x = mmin(running_min.x , compass.m.x);
    mmin->y = mmin(running_min.y , compass.m.y);
    mmin->z = mmin(running_min.z , compass.m.z);

    mmax->x = mmax(running_max.x , compass.m.x);
    mmax->y = mmax(running_max.y , compass.m.y);
    mmax->z = mmax(running_max.z , compass.m.z);

    snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}  val: {%+6d, %+6d, %+6d}",
      running_min.x, running_min.y, running_min.z,
      running_max.x, running_max.y, running_max.z,
      compass.m.x, compass.m.y, compass.m.z);
    delay(100);
  }
}
