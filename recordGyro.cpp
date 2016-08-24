#include "HardwarePi.h"
#include "Log.h"
#include <iostream>
#include <signal.h>

int bandwidth=3;
int samplerate=0;

HardwarePiInterfaceArduino interface;
LogRecordGyro pkt;


static volatile bool done=false;

void intHandler(int dummy) {
  done=true;
}



void setup() {
  interface.mpu.configure(0,0,bandwidth,samplerate);
  char buf[128];
  pkt.startPacket(1);
  pkt.write("Gyro Registers: ");
  pkt.endPacket(1);
  for(int i=0;i<sizeof(buf);i++) buf[i]=0;
  interface.mpu.readConfig(buf);
  for(int i=0;i<sizeof(buf);i+=16) {
    pkt.startPacket(1);
    pkt.write(buf+i,16);
    pkt.endPacket(1);
  }
  pkt.startDescribe(0);
  pkt.describe("t",pkt.t_float);
  pkt.describe("gx",pkt.t_i16);
  pkt.describe("gy",pkt.t_i16);
  pkt.describe("gz",pkt.t_i16);
  pkt.endDescribe(0);
  signal(SIGINT, intHandler); //trap SIGINT (Ctrl-C) so that we exit instead of crashing, thus running the destructors and flusing our logs
}

void loop() {
  int gyro[3];
  auto t=interface.time();
  interface.readGyro(gyro);
  pkt.startPacket(0);
  pkt.write((float)t);
  for(int i=0;i<3;i++) pkt.write(gyro[i]);
  pkt.endPacket(0);
  usleep(2000);
}

int main(int argc, char** argv) {
  if(argc>=2) bandwidth=atoi(argv[1]);
  if(argc>=3) samplerate=atoi(argv[2]);
  pkt.startPacket(1);
  pkt.write("Program arguments: ");
  pkt.endPacket(1);
  for(int i=0;i<argc;i++) {
    pkt.startPacket(1);
    pkt.write(i); 
    pkt.write(argv[i]);
    pkt.endPacket(1);
  }
  setup();
  while(!done) {
    loop();
  }
}
