#include "HardwarePi.h"
#include "Simulator.h"
#include "OpenLoopGuidance.h"
#include <iostream>

HardwarePiInterfaceArduino interface;

void setup() {
  printf("t,gx,gy,gz\n");
}

void loop() {
  int gyro[3];
  auto t=interface.time();
  interface.readGyro(gyro);
  printf("%f,%d,%d,%d\n",t,gyro[0],gyro[1],gyro[2]);
  usleep(2000);
}

int main() {
  setup();
  for(;;) {
    loop();
  }
}
