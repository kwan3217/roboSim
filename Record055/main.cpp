#include "BNO055.h"
//#include "HardwarePi.h"
#include "LogCSV.h"
#include "LogRawBinary.h"
#include "LogCCSDS.h"
#include "LogMulti.h"
#include "attach.h"
#include <iostream>
#include <signal.h>

int Argc; 
char** Argv;

BNO055 sensor;
LogCSV bnoconfigCSV("bnoconfig.csv",false);
LogRawBinary dumpTBZ("attach.tbz");
LogCCSDS pkt("packets.sds");
LogMulti<2> bnoconfig({&pkt,&bnoconfigCSV});
LogMulti<2> dump({&pkt,&dumpTBZ});

static volatile bool done=false;

void intHandler(int dummy) {
  done=true;
}

void setup() {
  //Open the I2C bus
  I2C_t bus=open("/dev/i2c-1",O_RDWR);
  if(bus<0) printf("Couldn't open bus: errno %d",errno);

  for(int i=0;i<Argc;i++) {
    bnoconfig.start(Log::Apids::argv,"CommandLineParameters");
    bnoconfig.write(Argv[i],"Parameter");
    bnoconfig.end();
  }
  if(!sensor.begin(bus)) {
	printf("Error starting BNO055 at line %d\n",errno);
  }
  char reg[256];
  for(size_t i=0;i<sizeof(reg);i++) reg[i]=0;
  if(!sensor.readConfig(reg)) {
	printf("Error reading BNO055 config at line %d\n",errno);
  }
  for(size_t i=0;i<sizeof(reg);i+=16) {
    bnoconfig.start(Log::Apids::gyroCfg,"BNO055Config");
    bnoconfig.write(reg+i,16,"registers");
    bnoconfig.end();
  }

  dumpAttach(dump,64);
}

void loop() {
  done=true;
}

int main(int argc, char** argv) {
  signal(SIGINT, intHandler); //trap SIGINT (Ctrl-C) so that we exit instead of crashing, thus running the destructors and flushing our logs
  Argc=argc;
  Argv=argv;
  setup();
  while(!done) {
    loop();
  }
}
