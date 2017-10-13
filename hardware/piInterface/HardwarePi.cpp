#include <time.h>
#include "HardwarePi.h"
#include <wiringPi.h>
#include <fcntl.h>
#include <termios.h>

/**
 * @copydoc Servo::write(int)
 * \internal
 * Implemented by writing to the correct registers in the Arduino serving as the odometer. The value written is a 16-bit unsigned integer
 * written little-endian to address 0x2C/0x2D or 0x2E/0x2F. This number is interpreted as the pulse width to use in 10us units. There is a software stop at 100 units and 200 units.
 */
void HardwarePiServoArduino::write(int n) {
  if(n<100) n=100;
  if(n>200) n=200;
  writeI2Creg_le<uint16_t>(bus,ADDRESS,0x2C+2*channel,n);
}

bool HardwarePiInterface::steerBoth(int16_t steeringCmd, int16_t throttleCmd) {
  char buf[7];
  if(steeringCmd<1000) steeringCmd=1000;
  if(steeringCmd>2000) steeringCmd=2000;
  if(throttleCmd<1000) throttleCmd=1000;
  if(throttleCmd<1000) throttleCmd=1000;
  int16_t checksum=steeringCmd ^ throttleCmd ^ 0x3217;
  buf[0]=0x30;
  writeBuf_le<int16_t>(buf,1,steeringCmd);
  writeBuf_le<int16_t>(buf,3,throttleCmd);
  writeBuf_le<int16_t>(buf,5,checksum);
  return writeI2C(bus,ODOMETER_ADDRESS,buf,7);
}

/**
 * @copydoc Interface::checkPPS()
 * \internal
 * Implemented using the LinuxPPS driver, which in turn implements RFC2783, Pulse-Per-Second API. If the interface epoch isn't valid
 * at the first PPS, then the epoch is set to the time of the PPS.
 *
 */
bool HardwarePiInterface::checkPPS(fp& t) {
  pps_info_t info;
  static const struct timespec timeout={0,0};
  time_pps_fetch(pps,PPS_TSFMT_TSPEC,&info,&timeout);
  t=ts2t(info.assert_timestamp-t0);
  bool has_new=(last_pps.tv_sec!=info.assert_timestamp.tv_sec) || (last_pps.tv_nsec!=info.assert_timestamp.tv_nsec);
  if(has_new) last_pps=info.assert_timestamp;
  return has_new;
}

/**
 * @copydoc Interface::time()
 * \internal
 * Implemented by reading the system clock, then subtracting off the epoch of the first time the clock was read.
 * Keeps timestamps and integers as long as possible so as not to lose precision.
 */
fp HardwarePiInterface::time() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME,&ts);
  return ts2t(ts-t0);
}

/** @copydoc Interface::button(int)
 *  \internal
 *  Since we use WiringPiSetupGpio(), we use the Broadcom numbers. This happens to match the numbers printed
 *  on our Pi header.
 */
bool HardwarePiInterface::button(int pin) {
  return 0==digitalRead(pin);
}

bool HardwarePiInterface::readOdometer(uint32_t &timeStamp, int32_t &wheelCount, uint32_t &dt) {
  if(ioctl(bus,I2C_SLAVE,ODOMETER_ADDRESS)<0) return false;
  char buf[0x10];
  buf[0]=0x00;
  if(1!=write(bus,buf,1)) return false;
  if(16!=read(bus,buf,16)) return false;
  wheelCount=readBuf_le<int32_t>(buf,0);
  dt        =readBuf_le<uint32_t>(buf,4);
  timeStamp =readBuf_le<uint32_t>(buf,8);
  cksumSent =readBuf_le<uint32_t>(buf,12);
  cksumCalc=wheelCount^dt^timeStamp^0x32171836;
  return cksumSent==cksumCalc;
}

bool HardwarePiInterface::readAcc(int16_t a[]) {
  return mpu.readGyro(a[0],a[1],a[2]);
}

bool HardwarePiInterface::readAcc(int16_t a[], int16_t& t) {
  return mpu.readGyro(a[0],a[1],a[2],t);
}

bool HardwarePiInterface::readGyro(int16_t g[], int16_t& t) {
  return mpu.readGyro(g[0],g[1],g[2],t);
}

bool HardwarePiInterface::readGyro(int16_t g[]) {
  return mpu.readGyro(g[0],g[1],g[2]);
}

bool HardwarePiInterface::readMag(int16_t b[]) {
  bool result=mpu.ak.read(b[1],b[0],b[2]);
  b[2]=-b[2];
  return result;
}

bool HardwarePiInterface::readGPS(double& t, double& lat, double& lon) {
  if(gps_read(&gps_data)>0 && old_gps_t!=gps_data.fix.time && gps_data.fix.mode>1) {
    lat=gps_data.fix.latitude;
    lon=gps_data.fix.longitude;
    t=gps_data.fix.time;
    return true;
  }
  return false;
}

bool HardwarePiInterface::readMPU(int16_t a[], int16_t g[], int16_t& t) {
  return mpu.readMPU(a[0],a[1],a[2],g[0],g[1],g[2],t);
}

HardwarePiInterface::HardwarePiInterface(Servo& Lsteering, Servo& Lthrottle):Interface(Lsteering,Lthrottle) {
  //Set epoch
  clock_gettime(CLOCK_REALTIME,&t0);
  //Setup for GPIO (for buttons)
  wiringPiSetupGpio();

  //Open PPS source
  ppsf=fopen("/dev/pps0","r");
  time_pps_create(fileno(ppsf), &pps);

  //Open the I2C bus
  bus=open("/dev/i2c-1",O_RDWR);
  if(bus<0) printf("Couldn't open bus: errno %d",errno);

  //Initialize the MPU9250
  mpu.begin(bus);

  //Initialize the GPS connection
  int result=gps_open(GPSD_SHARED_MEMORY,0,&gps_data);
  printf("Opened connection to gpsd, result %d (should be 0)\n",result);
}

HardwarePiInterface::~HardwarePiInterface() {
  time_pps_destroy(pps);
  fclose(ppsf);
  close(bus);
}

HardwarePiInterfaceArduino::HardwarePiInterfaceArduino():
HardwarePiInterface(hardSteering, hardThrottle), hardSteering(0),hardThrottle(1) {
  hardSteering.begin(bus);
  hardThrottle.begin(bus);
};

HardwarePiInterfaceArduino::~HardwarePiInterfaceArduino() {

}


