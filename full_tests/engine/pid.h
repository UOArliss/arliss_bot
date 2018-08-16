#ifndef PID_H
#define PID_H
#include <Arduino.h>

#define DEFAULT_SAMPLE_MS 1000

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
  upper =1000;
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

#endif
