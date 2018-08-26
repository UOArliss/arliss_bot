#include "pid.h"
#include "compass.h"
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h> //used by: motor shield
#include <Adafruit_MotorShield.h> //used by: motor shield
#include "utility/Adafruit_MS_PWMServoDriver.h" //used by: motor shield for DC motors

#define ECHOPIN 11
#define TRIGGERPIN 12
#define MAXDIST 300
#define FAST_SPEED 150
#define NORMAL_SPEED 125
#define TURN_SPEED 100
#define SLOW_SPEED 75
#define MAX_SPEED 255

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor* left_motor = AFMS.getMotor(1); //M1 port
Adafruit_DCMotor* right_motor = AFMS.getMotor(2); //M2 port

void setup(){

  /*begins with default frequency 1.6KHz*/ 
  AFMS.begin();
  pinMode(TRIGGERPIN , OUTPUT);
  pinMode(ECHOPIN , INPUT);
  Serial.begin(9600);

}


void loop(){
  uint8_t i;
  move_forward(150,150);
  int obj =0;
  /*PID loop needs to be refined, but basic functionality is present*/
  pid p = pid(1024, .5,.00001,.01);
  float speed;
  for(;;){
    move_forward();
    obj = obj_detection();
    if (obj){
      p.set_setpoint(speed/2);
      speed = p.compute(speed);
      left_motor->setSpeed(speed);
      right_motor->setSpeed(speed);
      Serial.println("Speed is ");
      Serial.println(speed);
      delay(50);
      while(obj = obj_detection() != 1){
      turn_left(speed/2, speed/2);
      move_forward(speed/2, speed/2);
      delay(500);
      turn_right(speed/2,speed/2);
      }
    } else {
    p.set_setpoint(150);
    speed = p.compute(speed);
    move_forward(speed,speed);
    delay(50);
    Serial.println("Speed is ");
    Serial.println(speed);

  }
}
}
/*
int obj_detection(){
  return 0;
}*/


int obj_detection(){
  digitalWrite(TRIGGERPIN , LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGERPIN , HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGERPIN , LOW);
  long duration = pulseIn(ECHOPIN, HIGH);

  long distance = duration / 58.2;

  if ( distance <= MAXDIST){
    return 1;
  } 
  return 0;
}

void turn_left(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  right_motor->run(FORWARD);
  left_motor->run(RELEASE);
}

void turn_right(int speed, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  left_motor->run(FORWARD);
  right_motor->run(RELEASE);	
}

void turn_straight(int speed, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  right_motor->run(FORWARD);
  left_motor->run(FORWARD);
}

void move_forward(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  left_motor->run(FORWARD);
  right_motor->run(FORWARD);
}

void move_backward(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  left_motor->run(BACKWARD);
  right_motor->run(BACKWARD);

}

void move_stop(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  left_motor->run(RELEASE);
  right_motor->run(RELEASE);
}

