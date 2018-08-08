//Arliss Bot v1
//
//ARDUINO (UNO) SETUP:
//======================
//Adafruit LSM303 Compass = VIN, GND, SDA (A4), SCL (A5)


/************* Libraries *************/
#include <Wire.h>
#include <SoftwareSerial.h> //for serial output
#include <math.h>//for M_PI
/** Using the Adafruit Sensor library found here: https://github.com/adafruit/Adafruit_Sensor **/
#include <Adafruit_Sensor.h>
/** Using the Adafruit LSM303 library found here: https://github.com/adafruit/Adafruit_LSM303DLHC **/
#include <Adafruit_LSM303_U.h>

/************* Globals *************/
/** Compass **/
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12334); //Create compass object with a unique id

/************* Setup *************/
void setup () {
	Serial.begin(115200);	//turn on serial monitor
	Serial.println("Compass Test\n");
    
	//Attempt to initialize compass
	if(!mag.begin())
	{
	    Serial.println("Could not detect compass...check wiring\n");
	    while(1);
	}
}

/************* Loop *************/
void loop() {
    //create compass event
    sensors_event_t event;
    mag.getEvent(&event); 
    
    float Pi = M_PI;
    
    //Calulating angle of vector y,x
    float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;

    //Normalize to 0-360
    if (heading < 0)
        heading = 360 + heading;
        
    Serial.println("Compass Heading: ");
    Serial.println(heading);
    delay(500);
}
