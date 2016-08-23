#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "robot.h"
#include "compassNeedle.h"
#include "Simulator.h" //quick-and-dirty solution to let the cheat compass take information from Simulator class

using namespace std;



compassNeedle::compassNeedle(Interface& Linterface, double h):
Controller(Linterface),heading(h)
{ }

void compassNeedle::guide(){
    desiredHeading = 0;
	headingChange = desiredHeading - heading;
	if(headingChange > 180){
		headingChange -= 360;
	}
	else if (headingChange < -180){
		headingChange += 360;
	}
}

void compassNeedle::control(){
  interface.steering.write(headingChange * double (50)/180+150);
}

void compassNeedle::updateTime(){
	double oldTime = epochTime;
	epochTime = interface.time();
	dt = epochTime - oldTime;
}

void compassNeedle::navigateCompass(){
	updateTime();
	int g[3];
	interface.readGyro(g);
	double actual = (double)g[2]/ 0x7FFF * 250;
	heading += actual * dt;
}

void compassNeedle::showVector() const {
  printf(",%6.2f,%6.2f,%6.2f\n",heading,desiredHeading,headingChange);
}

