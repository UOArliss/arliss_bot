#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#define G_SCALE 9.8 * 3 /*scale factor for x G's of gravitational force*/

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);


void setup(void)
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

}

void loop(void)
{
  if (is_launched())
    Serial.println("We are launched");
  else
    Serial.println("Not Launced");
  delay(500);
}

int is_launched(){
  sensors_event_t e;
  accel.getEvent(&e);
  return (e.acceleration.z > G_SCALE ) ? 1 : 0;
}
