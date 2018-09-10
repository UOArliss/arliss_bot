#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h> 
#include <Wire.h>
//#include <LSM303.h>
#include "compass.h"
#include <NewPing.h>
#include "TinyGPS++.h"
#include "pid.h"
/*defines for various constants*/
#define LAUNCH_TRIGGER_G 2000//8000
#define PARACHUTE_DEPLAY_M 2000
#define LAUNCH_DELAY 2000
#define BURN_DELAY 2000

#define GPS_PIN1 8
#define GPS_PIN2 7
#define GOAL_LAT 44.0465
#define GOAL_LON 123.0742

#define LEFT 1
#define RIGHT 2
#define SAFE_DIST 250
#define TURN_DIST 150
#define STOP_DIST 30
#define FAST_SPEED 150
#define NORMAL_SPEED 125
#define TURN_SPEED 100
#define SLOW_SPEED 75
#define MAX_SPEED 255
#undef _SS_MAX_RX_BUFFs
#define _SS_MAX_RX_BUFF 256
#define TURN_GIVE 10

/*ULTRASONIC PINS*/
#define TRIGGERPIN 9
#define ECHOPIN 9

/*object reactions*/
#define SAFE 0
#define SLOW 1
#define TURN 2
#define STOP 3
#define DEBUGGING_CODE 1
/*global objects from libraries*/

NewPing sonar(TRIGGERPIN, ECHOPIN, SAFE_DIST);
TinyGPSPlus gps;
LSM303 compass;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor* left_motor = AFMS.getMotor(1); //M1 port
Adafruit_DCMotor* right_motor = AFMS.getMotor(3); //M2 port
Adafruit_DCMotor* burn_port = AFMS.getMotor(2); //M2 port

SoftwareSerial gps_comm = SoftwareSerial(GPS_PIN1, GPS_PIN2);
double lat = 0;
double lon = 0;
double alt = 1000000;
unsigned long prevTime = 0;
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  //pinMode(GPS_PIN1, INPUT);
  //pinMode(GPS_PIN2, OUTPUT);
  /*launch code loop*/
  int has_launched = 0;
  /*detect launch*/
  while(!has_launched){
    has_launched = detect_launch();
    delay(50);
  }
  Serial.println("Launched!!");
  delay(LAUNCH_DELAY);
  /*detect when time to deploy the parachute*/
    //double lat;
    //double lon;
    int deployed = 0;
    //int current_alt = 1000000;/*init with a very large value*/
    /*begin gps detection of altitude deploy at macro set above*/
    gps_comm.begin(9600);
    while(!deployed){
      //Serial.print(" ");Serial.println(ss.available());
      pollGPS();
      if(alt <= PARACHUTE_DEPLAY_M){
        deployed = 1;
      }
    }
    Serial.println("Near landing!!!");
    delay(BURN_DELAY);
    Serial.println("Burning cord!");
/*   if (gps.location.isUpdated()){
          lat = gps.location.lat();
        lon = gps.location.lng();
            current_alt = gps.altitude.meters();
            Serial.print("Altitude ");Serial.println(current_alt);
            if (current_alt <= PARACHUTE_DEPLAY_M)
              deployed = 1;
        //delay(500);
      }
      //delay(200);*/
  /*want to be able to detect that we are on the ground*/


  /*we have deplayed at this point, now burn the parachute off*/
  burn_string();
  prevTime = millis();
}
int prevDir = 15;

int targetReached = 0;
void loop()
{
  if(targetReached == 1){
    delay(200);
    return;
  }
  pollGPS();
  if(millis() - prevTime > 250){
    prevTime = millis();
    compass.read();
    /*determine the course we need to go and set compass heading accoridingly*/
    double d_head = get_desired_heading(); /*desired heading*/
    double c_head = compass.heading();  /*current heading*/
  
    /*gets the current direction */
    int dir = calculate_direction(c_head, d_head, prevDir);
    prevDir = dir;
    /*set up speed*/
    pid p = pid(1024, .5,.00001,.01);
    if(gps.distanceBetween(lat,lon , GOAL_LAT,GOAL_LON) < 10){
      targetReached = 1;
    }
    //int obj = obj_detection();
    float speed = 0.0f;
    /*switch(obj){
      case SAFE:
        move_forward(MAX_SPEED,MAX_SPEED);
        Serial.println("SAFE");
        break;
      case SLOW:
        p.set_setpoint(speed/2);
        speed = p.compute(speed);
        move_forward(speed,speed);
        Serial.println("SLOW");
        break;
      case TURN:
        p.set_setpoint(TURN_SPEED);
        speed = p.compute(TURN_SPEED);
    
        while(obj = obj_detection() < 1){
        turn_left(speed,speed);
        delay(400);
        turn_right(speed,speed);
        }
        Serial.println("TURN");
        break;
        
      case STOP:
        p.set_setpoint(0);
        move_stop();
  
        while(obj = obj_detection() > 2){
        
        p.set_setpoint(NORMAL_SPEED);
        speed = p.compute(NORMAL_SPEED);
        }
        Serial.println("STOP");
        break;
       default:
        break;
    }
    */
  
    
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
  
    
  
    //delay(100);
  }
}

void pollGPS(){
  
  int cha = gps_comm.read();
      
      if(cha != -1){
        //Serial.print((char)cha);
        if(gps.encode((char)cha)){
          if(gps.location.isUpdated()){
            //Serial.println("");
            if(DEBUGGING_CODE == 1){Serial.print("LON: ");Serial.print(gps.location.lng());Serial.print("LAT: ");Serial.println(gps.location.lat());  }
             lat = gps.location.lat();
             lon = gps.location.lng();
          }
          if(gps.altitude.isUpdated()){
              if(DEBUGGING_CODE == 1){Serial.print("ALT: ");Serial.println(gps.altitude.meters());}
             alt = gps.altitude.meters();
          }
        }
      }
}

/*takes current heading and desired and give most effective way to turn left/right*/
int calculate_direction(double current , double desired, int PRV){
  double prov = desired - current;
  if (prov < 0)
    prov += 360;
  if (prov > 180 + TURN_GIVE + (PRV == LEFT ? -TURN_GIVE : 0)){
      return LEFT;
  }else if(prov < 180 - TURN_GIVE - (PRV == RIGHT ? -TURN_GIVE : 0)){
    return RIGHT;
  }else{
    return 15;
  }
}


int detect_launch(){
  compass.read();
  int16_t launch = (((int32_t)(abs(compass.a.z)>>4))*((int32_t)(abs(compass.a.z)>>4)) + ((int32_t)(abs(compass.a.y)>>4))*((int32_t)(abs(compass.a.y)>>4)) + ((int32_t)(abs((compass.a.x))>>4))*((int32_t)(abs((compass.a.x))>>4)))/1000;
  //Serial.print((compass.a.z/16)*(compass.a.z/16));Serial.print(' ');Serial.print((compass.a.y/16)*(compass.a.y/16));Serial.print(' ');Serial.println((compass.a.x/16)*(compass.a.x/16));
  Serial.println(launch);
  if (launch > LAUNCH_TRIGGER_G )
      return 1;
   return 0;
}

/*initializes the engine port and sends high signal to burn the string holding the parachute*/
void burn_string(){
  pinMode(10, OUTPUT);
  /*on testing this did not work look into increasing motor shield voltage output
  or another method of string burn*/
  digitalWrite(10, HIGH);
  //analogWrite(10,255);
  delay(1500);
  digitalWrite(10, LOW);

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
  
  if(distance == 0){
    return -1;
  }
  if(DEBUGGING_CODE == 1){Serial.println(distance);}
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
