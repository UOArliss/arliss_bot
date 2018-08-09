#include "pid.h"
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h> //used by: motor shield
#include <Adafruit_MotorShield.h> //used by: motor shield
#include "utility/Adafruit_MS_PWMServoDriver.h" //used by: motor shield for DC motors

#define FAST_SPEED 150
#define NORMAL_SPEED 125
#define TURN_SPEED 100
#define SLOW_SPEED 75
#define MAX_SPEED 255

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor* drive_motor = AFMS.getMotor(3); //M1 port
Adafruit_DCMotor* turn_motor = AFMS.getMotor(1); //M2 port

void setup(){

  /*begins with default frequency 1.6KHz*/ 
  AFMS.begin();


}


void loop(){
  uint8_t i;
  drive_motor->setSpeed(1);
  turn_straight();
  for(i = 2; i <= MAX_SPEED; i++){
    drive_motor->setSpeed(i);
    delay(100);/*1/10 of a second*/
  }
  drive_motor->run(RELEASE);
  float speed = 1;
  pid p = pid(NORMAL_SPEED, .5,.01,.0001);
  for(;;){
    drive_motor->setSpeed(speed);
    speed = p.compute(speed);
    delay(500);
    Serial.println("Speed is ");
    Serial.println(speed);
  }
  drive_motor->run(RELEASE);
}

void turn_left(){
  turn_motor->run(BACKWARD);
}

void turn_right(){
  turn_motor->run(FORWARD);
}

void turn_straight(){
  turn_motor->run(RELEASE);
}

void move_forward(){
  drive_motor->run(FORWARD);
}

void move_backward(){
  drive_motor->run(BACKWARD);
}

void move_stop(){
  drive_motor->run(RELEASE);
}
