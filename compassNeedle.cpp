#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "robot.h"
#include "compassNeedle.h"

using namespace std;



compassNeedle::compassNeedle(Interface& Linterface, double h):
Controller(Linterface),heading(h)
{ }

void compassNeedle::guide(){
  desiredHeading = 0;
  headingChange = desiredHeading - heading;
  if(headingChange > 180){
    headingChange -= 360;
  } else if (headingChange < -180){
    headingChange += 360;
  }
}

void compassNeedle::control(){
  steerCmd=headingChange * double (50)/180+150;
  interface.steering.write(steerCmd);
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
  zDN=g[2];
  yawRate = (double)g[2]/ 0x7FFF * 250;
  heading -= yawRate * dt; //Notice that since the Z axis points up, the right-hand vector rotation rule says that positive yaw rate DECREASES heading IE indicates a turn to the left
}

void compassNeedle::showVector() const {
  printf(",%f,%d,%f,%6.2f,%6.2f,%6.2f,%d\n",dt,zDN,yawRate,heading,desiredHeading,headingChange,steerCmd);
}

