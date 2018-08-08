//Arliss Bot v1
//
//ARDUINO (UNO) SETUP:
//=====================
//Ping sensor (HC-SR04) = 5V, GND, D11 (for both trigger & echo)
//Adafruit Ultimate GPS Logger Shield = D7 & D8 (Uses shield, so pins used internally) Make sure the logging switch is set to 'soft. serial'

/************* Libraries *************/
#include <Wire.h>
#include <NewPing.h> //for ping sensor

/************* Globals *************/
/** Ping sensor **/
#define TRIGGER_PIN 11
#define ECHO_PIN 11
#define MAX_DISTANCE_CM 250 //max dist we want to ping for
#define MAX_DISTANCE_IN (MAX_DISTANCE_CM/2.5) //max dist to ping for in inches
int sonarDistance;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);  //create ping object

/************* Setup *************/
void setup() {
	Serial.begin(115200);   //turn on serial monitor
        Serial.println("Ping Sensor Test\n");
}
/************* Loop *************/
void loop() {
	delay(50);
	Serial.println("Ping: ");
	Serial.println(sonar.ping_cm()); //send ping, get distance
	Serial.println("cm\n");
}
