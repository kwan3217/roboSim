#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
//#include "HeaderSimRobo.h"
#include "robot.h"
#include "compassNeedle.h"
#include "Simulator.h" //quick-and-dirty solution to let the cheat compass take information from Simulator class

using namespace std;



compassNeedle::compassNeedle(double h, double e, double n, Interface& Linterface):
heading(h), pos(e, n),interface(Linterface),headingChange(0),desiredHeading(0),
partCount(0), charsReceived(0), sentenceStart(false), wheelCount(0)
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
	if(nowpoint == 0){
		if(interface.button()) nowpoint = 1;
	} else {
		if(headingChange >= 300){
			interface.throttle.write(150);
			interface.steering.write(150);
			return;
		}
		interface.throttle.write(140);
		interface.steering.write(headingChange * double (50)/180+150);
	}
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

