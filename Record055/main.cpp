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

/** Figure the distance from the epoch to a given timespec using pure integer math
  @param ts given timespec
  @return a deltat timespec, where tv_sec and tv_nsec represent the difference between the timestamps
 */
inline struct timespec operator-(const struct timespec& a, const struct timespec& b) {
  struct timespec result;
  result.tv_sec=a.tv_sec-b.tv_sec;
  if(a.tv_nsec<b.tv_nsec) {
    result.tv_sec--;
    result.tv_nsec=(a.tv_nsec+1'000'000'000)-b.tv_nsec;
  } else {
    result.tv_nsec=a.tv_nsec-b.tv_nsec;
  }
  return result;
}

inline fp ts2t(struct timespec ts) {return ts.tv_sec+fp(ts.tv_nsec)/1'000'000'000.0;};

static volatile bool done=false;

void intHandler(int dummy) {
  done=true;
}

bool dumpConfig(bool print=false) {
  struct timespec ts0,ts1;
  clock_gettime(CLOCK_REALTIME,&ts0);
  if(!sensor.readConfig()) {
	printf("Error reading BNO055 config at line %d\n",errno);
	return false;
  }
  clock_gettime(CLOCK_REALTIME,&ts1);
  bnoreadout.write(uint32_t(ts0.tv_sec),"ts_sec");
  bnoreadout.write(uint32_t(ts0.tv_nsec),"ts_nsec");
  bnoconfig.start(Log::Apids::gyroCfg,"BNO055Config");
  bnoconfig.write(sensor.configBuf,sizeof(sensor.configBuf),"registers");
  bnoconfig.end();
  if(print) {
    for(size_t i=0;i<sizeof(sensor.configBuf);i+=16) {
	  printf("%02x,",i);
	  for(int j=0;j<16;j++) {
	    printf("%02x",sensor.configBuf[i+j]);
	  }
	  printf("\n");
	}
  }
  return true;
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
  dumpConfig(true);
  dumpAttach(dump,64);
}

void loop() {
  static const int sampleIntervalMS=10;
  static const int configIntervalMS=1000;
  static const int printIntervalMS=1000;
  static const int configIntervalN=configIntervalMS/sampleIntervalMS;
  static       int configIntervalI=6;
  static const int printIntervalN=printIntervalMS/sampleIntervalMS;
  static       int printIntervalI=0;
  struct timespec ts0,ts1,ts2,ts3;
  clock_gettime(CLOCK_REALTIME,&ts0);
  if(!sensor.sample()) printf("Problem reading sample\n");
  clock_gettime(CLOCK_REALTIME,&ts1);
  bnoreadout.start(Log::Apids::bno,"BNOData");
  bnoreadout.write(uint32_t(ts0.tv_sec),"ts_sec");
  bnoreadout.write(uint32_t(ts0.tv_nsec),"ts_nsec");
  int16_t raw[4];
  fp si[4];
  sensor.readAccRaw(raw);
  bnoreadout.write(raw[0],"ax");
  bnoreadout.write(raw[1],"ay");
  bnoreadout.write(raw[2],"az");
  sensor.readMagRaw(raw);
  bnoreadout.write(raw[0],"bx");
  bnoreadout.write(raw[1],"by");
  bnoreadout.write(raw[2],"bz");
  sensor.readGyroRaw(raw);
  bnoreadout.write(raw[0],"gx");
  bnoreadout.write(raw[1],"gy");
  bnoreadout.write(raw[2],"gz");
  sensor.readEulerRaw(raw);
  bnoreadout.write(raw[0],"eh");
  bnoreadout.write(raw[1],"er");
  bnoreadout.write(raw[2],"ep");
  sensor.readQuatRaw(raw);
  bnoreadout.write(raw[0],"qw");
  bnoreadout.write(raw[1],"qx");
  bnoreadout.write(raw[2],"qy");
  bnoreadout.write(raw[3],"qz");
  sensor.readLiaRaw(raw);
  bnoreadout.write(raw[0],"lax");
  bnoreadout.write(raw[1],"lay");
  bnoreadout.write(raw[2],"laz");
  sensor.readGrvRaw(raw);
  bnoreadout.write(raw[0],"gax");
  bnoreadout.write(raw[1],"gay");
  bnoreadout.write(raw[2],"gaz");
  bnoreadout.write(sensor.readTempRaw());
  bnoreadout.end();
  clock_gettime(CLOCK_REALTIME,&ts2);
  printIntervalI++;
  if(printIntervalI>=printIntervalN) {
    printf("%10d.%09d,",ts0.tv_sec,ts0.tv_nsec);
    sensor.readAcc(si);
    printf("%f,%f,%f,",si[0],si[1],si[2]);
    sensor.readMag(si);
    printf("%f,%f,%f,",si[0],si[1],si[2]);
    sensor.readGyro(si);
    printf("%f,%f,%f,",si[0],si[1],si[2]);
    sensor.readEuler(si);
    printf("%f,%f,%f,",si[0],si[1],si[2]);
    sensor.readQuat(si);
    printf("%f,%f,%f,%f,",si[0],si[1],si[2],si[3]);
    sensor.readLia(si);
    printf("%f,%f,%f,",si[0],si[1],si[2]);
    sensor.readGrv(si);
    printf("%f,%f,%f,",si[0],si[1],si[2]);
    printf("%f\n",sensor.readTemp());
    printIntervalI=0;
    clock_gettime(CLOCK_REALTIME,&ts3);
    printf("dt1=%f,dt2=%f,dt3=%f\n",ts2t(ts1-ts0),ts2t(ts2-ts0),ts2t(ts3-ts0));
  }
  configIntervalI++;
  if(configIntervalI>=configIntervalN) {
    dumpConfig(false);	
    configIntervalI=0;
  }
  usleep(sampleIntervalMS*1000);
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
