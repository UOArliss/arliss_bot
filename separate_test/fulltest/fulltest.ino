#include <Adafruit_MotorShield.h>


#ifndef PID_H
#define PID_H
#define DEFAULT_SAMPLE_MS 1000
#include <Arduino.h>
#include <Wire.h> //used by: motor shield
#include <Adafruit_MotorShield.h> //used by: motor shield
#include "utility/Adafruit_MS_PWMServoDriver.h" //used by: motor shield for DC motors

class pid{

  public:
    pid();
    pid(float dval, float p, float i, float d);
    /*functions to get/view the proportion/integral/derivative terms*/
    inline float get_kp(){ return this->Kp;}
    inline float get_ki(){ return this->Ki;}
    inline float get_kd(){ return this->Kd;}

    /*in case need to change the sample values on the fly...
    note these probably wont be used*/
    inline void set_kp(float p){ this->Kp = p;}
    inline void set_ki(float i){ this->Ki = i;}
    inline void set_kd(float d){ this->Kd = d;}

    

    /*change sample rate in ms, default is 1000 = 1sec*/
    void set_sample_rate(float rate);

    float compute(float input);

  private:
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float derivative;

    unsigned int prev_time;
    /*desired value*/
    float setpoint;
    float sample_rate;

    float last_error;
    float error;

    /*bounds*/
    float upper;
    float lower;

    /*private functions*/
    void limit(float* output);




};


pid::pid(float dval, float p, float i, float d){

  Kp = p;
  Ki = i;
  Kd = d;

  sample_rate = DEFAULT_SAMPLE_MS;
  prev_time = 0;
  setpoint = dval;

  /*set upper and lower*/
  upper = 255;
  lower = 0;
}

void pid::limit(float* output){
  if (*output > upper)
    *output = upper;
  else if (*output < lower)
    *output = lower;
}

float pid::compute(float input){
  /*get current time*/
  /*need to include header*/
  unsigned int now = millis();
  
  error = setpoint - input;

  if (prev_time == 0){
    derivative = 0;
    integral = 0;
  } else{
    int dtime = now - prev_time;
    derivative = error / dtime;
    integral += error * dtime;
  }

  last_error = error;
  prev_time = now;
  float kp = Kp * error;
  float ki = ki * integral;
  float kd = kd * derivative;

  float output = kp + ki + kd;
  limit(&output);
  return output;

}

void pid::set_sample_rate(float rate){

  if ( rate > 0){
    float r = rate / sample_rate;

    Ki = Ki*r;
    Kd = Kd / r;
    sample_rate = rate;
  }
}

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
    p.compute(speed);
    delay(500);
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

#endif
