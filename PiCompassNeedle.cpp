#include <iostream>
#include <cmath>
#include "robot.h"
#include "HardwarePi.h"
#include "compassNeedle.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;

int main() {
  HardwarePiInterfaceArduino interface;
  double heading;
  compassNeedle robo(interface,90);
  interface.throttle.write(150);
  interface.steering.write(150);
  while(true) {
    interface.update(0.05);
    robo.navigate();
    robo.guide();
    robo.control();
  }
  return 0;
}

