#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h> 
#include <Wire.h>
//#include <LSM303.h>
#include "compass.h"
#include <NewPing.h>
#include "TinyGPS++.h"
#include "pid.h"
/*defines for various constants*/
#define LAUNCH_TRIGGER_G 2
#define PARACHUTE_DEPLAY_M 2000

#define GPS_PIN1 4
#define GPS_PIN2 3
#define GOAL_LAT 49999
#define GOAL_LON 595

#define LEFT 1
#define RIGHT 2
#define SAFE_DIST 300
#define TURN_DIST 150
#define STOP_DIST 30
#define FAST_SPEED 150
#define NORMAL_SPEED 125
#define TURN_SPEED 100
#define SLOW_SPEED 75
#define MAX_SPEED 255


/*ULTRASONIC PINS*/
#define TRIGGERPIN 11
#define ECHOPIN 11

enum distances { SAFE, SLOW ,TURN,  STOP};
/*object reactions*/
#define SAFE 0
#define SLOW 1
#define TURN 2
#define STOP 3

/*global objects from libraries*/

NewPing sonar(TRIGGERPIN, ECHOPIN, SAFE_DIST);
TinyGPSPlus gps;
LSM303 compass;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor* left_motor = AFMS.getMotor(1); //M1 port
Adafruit_DCMotor* right_motor = AFMS.getMotor(3); //M2 port
Adafruit_DCMotor* burn_port = AFMS.getMotor(2); //M2 port



void setup()
{
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();

  /*launch code loop*/
  int has_launched = 0;
  /*detect launch*/
  while(!has_launched){
    has_launched = detect_launch();
    delay(500);
  };

  /*detect when time to deploy the parachute*/
    double lat;
    double lon;
    int deployed = 0;
    int current_alt = 1000000;/*init with a very large value*/
    /*begin gps detection of altitude deploy at macro set above*/
    SoftwareSerial ss(GPS_PIN1, GPS_PIN2);
    while(!deployed){
      while(ss.available() > 0){
        gps.encode(ss.read());
      
      lat = gps.location.lat();
      lon = gps.location.lng();

      if (gps.location.isUpdated()){
          current_alt = gps.altitude.meters();
          if (current_alt <= PARACHUTE_DEPLAY_M)
            deployed = 1;
      }
    }
      delay(200);
    }

  /*we have deplayed at this point, now burn the parachute off*/
  burn_string();
  float speed = MAX_SPEED;
  int obj = 0;
}

void loop()
{
  
  compass.read();
  /*determine the course we need to go and set compass heading accoridingly*/
  double d_head = get_desired_heading(); /*desired heading*/
  double c_head = compass.heading();  /*current heading*/

  /*gets the current direction */
  int dir = calculate_direction(c_head, d_head);

  /*set up speed*/
  pid p = pid(1024, .5,.00001,.01);
  
  int obj = obj_detection();
  float speed = 0.0f;
  switch(obj){
    case SAFE:
      move_forward(MAX_SPEED,MAX_SPEED);
      break;
    case SLOW:
      p.set_setpoint(speed/2);
      speed = p.compute(speed);
      move_forward(speed,speed);
      break;
    case TURN:
      p.set_setpoint(TURN_SPEED);
      speed = p.compute(TURN_SPEED);

      while(obj = obj_detection() < 1){
      turn_left(speed,speed);
      delay(400);
      turn_right(speed,speed);
      }
      break;
    case STOP:
      p.set_setpoint(0);
      move_stop();

      while(obj = obj_detection() > 2){
      
      p.set_setpoint(NORMAL_SPEED);
      speed = p.compute(NORMAL_SPEED);
      }
      break;
  }

  if ( dir == LEFT){
    speed = TURN_SPEED;
    turn_left(speed,speed);

  } else if (dir == RIGHT) {
    speed = TURN_SPEED;
    turn_right(speed,speed);

  } else {
    speed = MAX_SPEED;
    turn_straight(speed,speed);
  }

  

  delay(100);
}

/*takes current heading and desired and give most effective way to turn left/right*/
int calculate_direction(double current , double desired){
  double prov = desired - current;
  if (prov < 0)
    prov += 360;
  if (prov > 180)
      return LEFT;
  return RIGHT;
}


int detect_launch(){
  compass.read();
  int launch = (compass.a.z >> 4) / 1000;
  if (launch > LAUNCH_TRIGGER_G )
      return 1;
   return 0;
}

/*initializes the engine port and sends high signal to burn the string holding the parachute*/
void burn_string(){
  /*on testing this did not work look into increasing motor shield voltage output
  or another method of string burn*/
  AFMS.begin();
  burn_port->setSpeed(255);
  burn_port->run(FORWARD);
  delay(1000);
  burn_port->setSpeed(0);
  burn_port->run(RELEASE);
}

double get_desired_heading(){
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double heading =  gps.courseTo(lat,lon , GOAL_LAT,GOAL_LON);
  return heading;
}




/**************
  Movement Functions
**************/
void turn_left(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  right_motor->run(FORWARD);
  left_motor->run(RELEASE);
}

void turn_right(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  left_motor->run(FORWARD);
  right_motor->run(RELEASE);  
}

void turn_straight(int speed1, int speed2){
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

void move_stop(){
  left_motor->run(RELEASE);
  right_motor->run(RELEASE);
}

/*function to detect if obj is within range*/
int obj_detection(){
  /*digitalWrite(TRIGGERPIN , LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGERPIN , HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGERPIN , LOW);
  long duration = pulseIn(ECHOPIN, HIGH);

  long distance = duration / 58.2;
  */
  long distance = sonar.ping_cm();
  if (distance >= SAFE_DIST)
    return SAFE;
  if (distance < SAFE_DIST && distance > TURN_DIST)
    return SLOW;
  if (distance < TURN_DIST && distance > STOP_DIST)
    return TURN;
  if (distance <= STOP_DIST)
    return STOP;
  return -1;
}

