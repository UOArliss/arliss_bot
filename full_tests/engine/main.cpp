#include "pid.h"
#include <cstdio>


int main(int argc, char** argv){
	float speed = 1;
	pid p = pid(150, .5,.01,.0001);
  	for(;;){
    
    printf("speed is %f\n",speed);
    speed = p.compute(speed);
  }

	return 0;
}