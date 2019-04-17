#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h> 
#include <Wire.h>
#include "compass.h"
#include "TinyGPS++.h"
#include "pid.h"

#define LAUNCH_TRIGGER_G 8000//8000
#define PARACHUTE_DEPLAY_M 1489
#define LAUNCH_DELAY 60000
#define BURN_DELAY 5000
#define GROUND_ALT 1200
#define GPS_PIN1 8
#define GPS_PIN2 7       
#define GOAL_LAT 40.88
#define GOAL_LON -119.12

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
#define TURN_GIVE 25

/*ULTRASONIC PINS*/
 #define TRIGGERPIN 9
#define ECHOPIN 9

/*object reactions*/
#define SAFE 0
#define SLOW 1
#define TURN 2
#define STOP 3
#define DEBUGGING_CODE 0
/*global objects from libraries*/
TinyGPSPlus gps;
LSM303 compass;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor* left_motor = AFMS.getMotor(2); //M1 port
Adafruit_DCMotor* right_motor = AFMS.getMotor(1); //M2 port

SoftwareSerial gps_comm = SoftwareSerial(GPS_PIN1, GPS_PIN2);
double lat = 0;
double lon = 0;
double alt = 1;
unsigned long prevTime = 0;
void setup()
{
  Serial.begin(9600);
  Wire.begin();
    AFMS.begin();

  
  compass.init();
  compass.enableDefault();
  int has_launched = 0;
  /*detect launch*/
  gps_comm.begin(9600);
  unsigned long ptime = millis();
  while(!has_launched){
    pollGPS();
    if(millis() - ptime > 250){
    //delay(50);
    ptime = millis();
    has_launched = detect_launch();
    }
  }
  Serial.println("Launched!!");
  delay(LAUNCH_DELAY);
    int deployed = 0;
    
    while(!deployed){
      //Serial.print(" ");Serial.println(ss.available());
      pollGPS();
      if(alt <= PARACHUTE_DEPLAY_M){
        deployed = 1;
      }
    }
    Serial.println("Near landing!!!");
    //delay(BURN_DELAY);
    Serial.println("Burning cord!");

  /*we have deplayed at this point, now burn the parachute off*/
  //burn_string();
  prevTime = millis();
  pid p = pid(1024, .5,.00001,.01);
}
int prevDir = 35;
int targetReached = 0;
void loop()
{
  if(targetReached == 1){
    delay(200);
    Serial.println("DONTRNTNEHE");
    return;
  }
  pollGPS();
  if(millis() - prevTime > 250){
    Serial.println("in timer");
    prevTime = millis();
    compass.read();
    /*determine the course we need to go and set compass heading accoridingly*/
    double d_head = get_desired_heading(); /*desired heading*/
    Serial.println("desired heading is");
    Serial.println(d_head);
    double c_head = compass.heading();   /*current heading*/
    Serial.println("calc heading is");
    Serial.println(c_head);
    /*gets the current direction */
    int dir = calculate_direction(c_head, d_head, prevDir);
    
    /*set up speed*/
    
    if(gps.distanceBetween(lat,lon , GOAL_LAT,GOAL_LON) < 10){
      targetReached = 1;
    }

    float speed = 0.0f;
    
    if ( dir == LEFT){
      if(dir != prevDir){ 
        Serial.println("LEFT");
        speed = NORMAL_SPEED;
        turn_left(speed,speed);
      }
    } else if (dir == RIGHT) {
      if(dir != prevDir){ 
        Serial.println("RIGHT");
        speed = NORMAL_SPEED;
        turn_right(speed,speed);
      }
    } else {
      if(dir != prevDir){  
            Serial.println("Straight");
      speed = FAST_SPEED;
      turn_straight(speed,speed);
      }
    }
  prevDir = dir;

  

  
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

int check_alt(){
    //poll_gps();
    if( alt > GROUND_ALT + 400)
      return 1;
    return 0; 
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
  if (launch > LAUNCH_TRIGGER_G || check_alt() == 1)
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
void turn_right(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  right_motor->run(FORWARD);
  left_motor->run(RELEASE);
}

void turn_left(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  left_motor->run(BACKWARD);
  right_motor->run(RELEASE);  
}

void turn_straight(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  right_motor->run(FORWARD);
  left_motor->run(BACKWARD);
}

void move_forward(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  left_motor->run(BACKWARD);
  right_motor->run(FORWARD);
}

void move_backward(int speed1, int speed2){
  left_motor->setSpeed(speed1);
  right_motor->setSpeed(speed2);
  left_motor->run(FORWARD);
  right_motor->run(BACKWARD);

}

void move_stop(){
  left_motor->run(RELEASE);
  right_motor->run(RELEASE);
}

/*function to detect if obj is within range
int obj_detection(){
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
}*/

