




#define ECHOPIN 11
#define TRIGGERPIN 11
#define MAXDIST 300 /*distance in cm of obj detection*/

void setup()
{
	pinMode(TRIGGERPIN , OUTPUT);
	pinMode(ECHOPIN , INPUT);

}

void loop(){
	  Serial.begin(9600);
	digitalWrite(TRIGGERPIN , LOW);
	delayMicroseconds(2);
	digitalWrite(TRIGGERPIN , HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIGGERPIN , LOW);
	long duration = pulseIn(ECHOPIN, HIGH);

	long distance = duration / 58.2;
  Serial.println(distance);
	if ( distance <= MAXDIST){

		Serial.println("Object in range"); 
	} else {
		Serial.println("Object not in range");
	}
	delay(50);
}
