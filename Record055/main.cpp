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
LogMulti<1> bnoreadout({&pkt});
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
    printf("%02x,",i);
    for(int j=0;j<16;j++) {
	  printf("%02x",reg[i+j]);
	}
	printf("\n");
  }
  dumpAttach(dump,64);
}

void loop() {
  struct timespec ts0,ts1;
  clock_gettime(CLOCK_REALTIME,&ts0);
  sensor.sample();
  clock_gettime(CLOCK_REALTIME,&ts1);
  bnoreadout.start(Log::Apids::bno,"BNOData");
  bnoreadout.write(uint32_t(ts0.tv_sec),"ts_sec");
  bnoreadout.write(uint32_t(ts0.tv_nsec),"ts_nsec");
  printf("%10d.%09d,",ts0.tv_sec,ts0.tv_nsec);
  int16_t raw[4];
  fp si[4];
  sensor.readAccRaw(raw);
  sensor.readAcc(si);
  bnoreadout.write(raw[0],"ax");
  bnoreadout.write(raw[1],"ay");
  bnoreadout.write(raw[2],"az");
  printf("%f,%f,%f,",si[0],si[1],si[2]);
  sensor.readMagRaw(raw);
  sensor.readMag(si);
  bnoreadout.write(raw[0],"bx");
  bnoreadout.write(raw[1],"by");
  bnoreadout.write(raw[2],"bz");
  printf("%f,%f,%f,",si[0],si[1],si[2]);
  sensor.readGyroRaw(raw);
  sensor.readGyro(si);
  bnoreadout.write(raw[0],"gx");
  bnoreadout.write(raw[1],"gy");
  bnoreadout.write(raw[2],"gz");
  printf("%f,%f,%f,",si[0],si[1],si[2]);
  sensor.readEulerRaw(raw);
  sensor.readEuler(si);
  bnoreadout.write(raw[0],"eh");
  bnoreadout.write(raw[1],"er");
  bnoreadout.write(raw[2],"ep");
  printf("%f,%f,%f,",si[0],si[1],si[2]);
  sensor.readQuatRaw(raw);
  sensor.readQuat(si);
  bnoreadout.write(raw[0],"qw");
  bnoreadout.write(raw[1],"qx");
  bnoreadout.write(raw[2],"qy");
  bnoreadout.write(raw[3],"qz");
  printf("%f,%f,%f,%f,",si[0],si[1],si[2],si[3]);
  sensor.readLiaRaw(raw);
  sensor.readLia(si);
  bnoreadout.write(raw[0],"lax");
  bnoreadout.write(raw[1],"lay");
  bnoreadout.write(raw[2],"laz");
  printf("%f,%f,%f,",si[0],si[1],si[2]);
  sensor.readGrvRaw(raw);
  sensor.readGrv(si);
  bnoreadout.write(raw[0],"gax");
  bnoreadout.write(raw[1],"gay");
  bnoreadout.write(raw[2],"gaz");
  printf("%f,%f,%f,",si[0],si[1],si[2]);
  bnoreadout.write(sensor.readTempRaw());
  printf("%f\n",sensor.readTemp());
  bnoreadout.end();
  usleep(100000);
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
