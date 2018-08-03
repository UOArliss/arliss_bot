//Arliss Bot v1
//
//ARDUINO (UNO) SETUP:
//


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


//Create the motor shield objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Create our motors
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);

void setup() {
	AFMS.begin();

	leftMotor->setSpeed(200);
	rightMotor->setSpeed(200);
}

void loop() {
	delay(50);
	leftMotor->run(FORWARD);
	rightMotor->run(FORWARD);
	delay(1000);
	//Slow Down
	leftMotor->setSpeed(150);
	rightMotor->setSpeed(150);
	leftMotor->setSpeed(75);
	rightMotor->setSpeed(75);
	leftMotor->setSpeed(50);
	rightMotor->setSpeed(50);

	//Stop
	leftMotor->run(RELEASE); 
	rightMotor->run(RELEASE); 

	//Roll back
	leftMotor->run(BACKWARD);
	rightMotor->run(BACKWARD);
	
	delay(300);
	leftMotor->run(RELEASE); 
	rightMotor->run(RELEASE); 

}
