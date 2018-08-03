//Arliss Bot v1
//
//ARDUINO (UNO) SETUP:
//======================
//Adafruit MotorShield v2 = M1,M3

/******************** Libraries ********************/
#include <Wire.h> //used by: motor shield
#include <Adafruit_MotorShield.h> //used by: motor shield
#include "utility/Adafruit_MS_PWMServoDriver.h" //used by: motor shield for DC motors


/******************** Globals ********************/
//Create the motor shield objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Create our motors
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);



/******************** Setup ********************/
void setup() {
	AFMS.begin();

	//Set default motor speed
	leftMotor->setSpeed(200);
	rightMotor->setSpeed(200);
}


/******************** Loop ********************/
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
