#include "HardwarePi.h"
#include "Simulator.h"
#include "OpenLoopGuidance.h"
#include <iostream>

HardwarePiInterfaceArduino interface;
//Simulator interface;
double t[]           {0,  0,  2,  5,  7,  9,  99999999};
char servoChannel[] {'T','S','T','S','S','T','T'};
int servoCommand[]  {150,150,140,200,150,150,150};
OpenLoopGuidance guide(interface,t,servoChannel,servoCommand);

int ouf;
char buf[128];

void setup() {
  //The following is only necessary if you don't trust the compiler
  //and setup code to initialize buf to 0
//  for(int i=0;i<sizeof(buf);i++) buf[i]=0;
  ouf=fileno(stdout);
//  int whoami=interface.mpu.whoami();
//  printf("whoami: %02x",whoami);
//  while(guide.time()<15) {
  //First few bytes written out are gyro configuration. Gyro is
  //configured in the constructor for HardwarePiInterfaceArduino,
  //so the configuration is finished by this point.
  //First are registers 0x13 to 0x1F inclusive (13 bytes)
  readI2Creg(interface.bus,MPUI2C::ADDRESS,0x13,buf+0x13,13);
  //Next are registers 0x23 to 0x3A inclusive (24 bytes)
  readI2Creg(interface.bus,MPUI2C::ADDRESS,0x23,buf+0x23,24);
  //Next are registers 0x67 to 0x6C inclusive (6 bytes)
  readI2Creg(interface.bus,MPUI2C::ADDRESS,0x67,buf+0x67,6);
  //Next are registers 0x72 to 0x75 inclusive (4 bytes)
  readI2Creg(interface.bus,MPUI2C::ADDRESS,0x72,buf+0x72,4);
  //Next are registers 0x77 to 0x78 inclusive (2 bytes)
  readI2Creg(interface.bus,MPUI2C::ADDRESS,0x77,buf+0x77,2);
  //Next are registers 0x7A to 0x7B inclusive (2 bytes)
  readI2Creg(interface.bus,MPUI2C::ADDRESS,0x7A,buf+0x7A,2);
  //Finally are registers 0x7D to 0x7E inclusive (2 bytes)
  readI2Creg(interface.bus,MPUI2C::ADDRESS,0x7D,buf+0x7D,2);
  //Write the whole buffer at once, so that any offset into the first
  //128 bytes is the register number at this time
  write(ouf,buf,128);
}

void loop() {
//	interface.update(0.05);
//  guide.control();
//	std::cout << interface.time() << ',';
//	interface.showVector();
//	std::cout << std::endl;
  int16_t gx,gy,gz;
  interface.mpu.readGyro(gx,gy,gz);
  writeBuf_le(buf,0,uint32_t(interface.time()*1'000'000));
  writeBuf_le(buf,4,gx);
  writeBuf_le(buf,6,gy);
  writeBuf_le(buf,8,gz);
  write(ouf,buf,10);
  usleep(2000);
}

int main() {
  setup();
  for(;;) {
    loop();
  }
}
