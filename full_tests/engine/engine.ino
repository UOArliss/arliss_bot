#include "pid.h"
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

Adafruit_DCMotor* drive_motor = AFMS.getMotor(1); //M1 port
Adafruit_DCMotor* turn_motor = AFMS.getMotor(2); //M2 port

void setup(){

  /*begins with default frequency 1.6KHz*/ 
  AFMS.begin();
  pinMode(TRIGGERPIN , OUTPUT);
  pinMode(ECHOPIN , INPUT);
  Serial.begin(9600);

}


void loop(){
  uint8_t i;
  move_forward();
  
  drive_motor->setSpeed(150);
  turn_motor->setSpeed(150);
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
      drive_motor->setSpeed(speed);
      turn_motor->setSpeed(speed);
      Serial.println("Speed is ");
      Serial.println(speed);
      delay(50);
    } else {
      p.set_setpoint(1024);
    drive_motor->setSpeed(speed);
    turn_motor->setSpeed(speed);
    speed = p.compute(speed);
    delay(50);
    Serial.println("Speed is ");
    Serial.println(speed);
  }
}
}

int obj_detection(){
  return 0;
}

/*
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
}*/

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
  turn_motor->run(FORWARD);
}

void move_backward(){
  drive_motor->run(BACKWARD);
}

void move_stop(){
  drive_motor->run(RELEASE);
}

