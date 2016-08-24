#include "HardwarePi.h"
#include "Simulator.h"
#include "OpenLoopGuidance.h"
#include <iostream>

int bandwidth=3;
int samplerate=0;

HardwarePiInterfaceArduino interface(3,0);

void setup() {
  interface.mpu.begin(0,0,bandwidth,samplerate);
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

int main(int argc, char** argv) {
  if(argc>=2) bandwidth=atoi(argv[1]);
  if(argc>=3) bandwidth=atoi(argv[2]);
  setup();
  for(;;) {
    loop();
  }
}
