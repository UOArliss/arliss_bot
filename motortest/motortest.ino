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


bool kyblat = false;
/******************** Setup ********************/
void setup() {
	AFMS.begin();

	//Set default motor speed
	leftMotor->setSpeed(255);
	rightMotor->setSpeed(255);
  kyblat = true;
}


/******************** Loop ********************/
void loop() {
  if(kyblat == true){
  	delay(50);
    leftMotor->setSpeed(255);
    rightMotor->setSpeed(255);
    
  	leftMotor->run(FORWARD);
  	rightMotor->run(FORWARD);
  	delay(10000);
  	//Slow Down
  	leftMotor->setSpeed(255);
  	rightMotor->setSpeed(255);
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    delay(200);
  	leftMotor->setSpeed(255);
  	rightMotor->setSpeed(255);
   leftMotor->run(FORWARD);
   rightMotor->run(FORWARD);
    delay(1000);
  	leftMotor->setSpeed(50);
  	rightMotor->setSpeed(50);
   leftMotor->run(FORWARD);
   rightMotor->run(FORWARD);
    delay(200);
  	//Stop
  	leftMotor->run(RELEASE); 
  	rightMotor->run(RELEASE); 
    delay(200);
  	//Roll back
    leftMotor->setSpeed(255);
  	leftMotor->run(BACKWARD);
  	rightMotor->run(BACKWARD);
    
  	delay(3000);
  	leftMotor->run(RELEASE); 
  	rightMotor->run(RELEASE); 
    kyblat = false;
  }
}
