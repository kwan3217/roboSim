#include "HardwarePi.h"
#include "Simulator.h"
#include "OpenLoopGuidance.h"
#include <iostream>

HardwarePiInterfaceArduino interface;

void setup() {
  char buf[128];
  for(int i=0;i<sizeof(buf);i++) buf[i]=0;
  interface.mpu.readConfig(buf);
  for(int i=0;i<sizeof(buf);i+=16) {
    printf("%02x: ",i);
    for(int j=0;j<16;j+=4) {
      for(int k=0;k<4;k++) {
        printf("%02x",buf[i+j+k]);
      }
//      printf(" ");
    }
    printf("\n");
  }
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
