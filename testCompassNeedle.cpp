#include <iostream>
#include <cmath>
#include "robot.h"
#include "Simulator.h"
#include "compassNeedle.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;


int main() {
  cout << "time,pos.easting,pos.northing,speed,heading,kappa,,nav.heading,desiredHeading,headingChange" << endl; //.csv headers
  Simulator roboSim(90);
  double heading;
  roboSim.cheatHeading(heading);
  compassNeedle robo(roboSim,heading);
  roboSim.throttle.write(140);
  roboSim.steering.write(150);
  while(true) {
    roboSim.update(0.05);
    robo.navigate();
    robo.guide();
    robo.control();
    if(roboSim.time() > 10) break;
    cout << roboSim.time() << ",";
    roboSim.showVector();
    robo.showVector();
  }
  cout << "\nEND";
  return 0;
}

