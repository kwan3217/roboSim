#include <time.h>
#include "HardwarePi.h"
#include <wiringPi.h>

/**
 * @copydoc Servo::write(int)
 * \internal
 * Implemented using the ServoBlaster user-space driver. That driver program sets up a named pipe in /dev/servoblaster. Commands
 * to it are written in the form of <channel>=<value> where channel is a number between 0 and 7, and value is the pulse time of the servo
 * command, in 10us units.
 */
void HardwarePiServoBlaster::write(int n) {
  fprintf(ouf,"%d=%d\n",channel,n);
  fflush(ouf);
}

/**
 * @copydoc Servo::write(int)
 * \internal
 * Implemented by writing to the correct registers in the Arduino serving as the odometer. The value written is a 16-bit unsigned integer
 * written little-endian to address 0x2C/0x2D or 0x2E/0x2F. This number is interpreted as the pulse width to use in 10us units.
 */
void HardwarePiServoArduino::write(int n) {
  writeI2Creg_le<uint16_t>(bus,ADDRESS,0x2C+2*channel,n);
}

/**
 * @copydoc Interface::checkPPS()
 * \internal
 * Implemented using the LinuxPPS driver, which in turn implements RFC2783, Pulse-Per-Second API. If the interface epoch isn't valid
 * at the first PPS, then the epoch is set to the time of the PPS.
 *
 */
double HardwarePiInterface::checkPPS() {
//  pps_info_t info;
//  static const struct timespec timeout={0,0};
//  time_pps_fetch(pps,PPS_TSFMT_TSPEC,&info,&timeout);
//  double t=ts2t(info.assert_timestamp);
//  if(t0<t) t0=t;
//  return t-t0;
	return 0; //Comment out PPS stuff because no PPS source is plugged in
}

/** Makes sure that GPS buffer has data to read. If there is still data in the buffer that hasn't been spooled out, return immediately. If
 * we have read to the end of the buffer, then try to read a full buffer, but set the length of data left to the amount we actually read.
 * The file has been opened with O_NONBLOCK so it should return immediately even if there isn't a full buffer worth of data to read.
 */
void HardwarePiInterface::fillGpsBuf() {
//  if(gpsPtr<gpsLen) return;
//  gpsLen=fread(gpsBuf,1,sizeof(gpsBuf),gpsf);
//  gpsPtr=0;
}

/**
 * @copydoc Interface::checkNavChar()
 * \internal
 * Implemented with an internal buffer. First, the buffer is filled if necessary with fillGpsBuf. Then, return true if there is more than 1 byte in the buffer.
 */
bool HardwarePiInterface::checkNavChar() {
  fillGpsBuf(); //Make sure the buffer has data in it
  return gpsLen!=0;
}

/**
 * @copydoc Interface::readChar()
 * \internal
 * Implemented with an internal buffer. First, the buffer is filled if necessary with
 * fillGpsBuf. Then, return the character pointed to by the buffer read pointer, and
 * increment that pointer. If checkNavChar would return false, this will read the
 * character at index 0 in the buffer, which is old data.
 */
char HardwarePiInterface::readChar() {
  fillGpsBuf();
  char result=gpsBuf[gpsPtr];
  gpsPtr++;
  return result;
}

/**
 * @copydoc Interface::time()
 * \internal
 * Implemented by reading the system clock, then subtracting off the epoch of the first time the clock was read. If this *is* the first time
 *  the clock was read, records this time as the epoch in t0.
 */
double HardwarePiInterface::time() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME,&ts);
  double t=ts2t(ts);
  if(t0<0) t0=t;
  return t-t0;
}

/** @copydoc Interface::button(int)
 *  \internal
 *  Since we use WiringPiSetupGpio(), we use the Broadcom numbers. This happens to match the numbers printed
 *  on our Pi header.
 */
bool HardwarePiInterface::button(int pin) {
  return 0==digitalRead(pin);
}

void HardwarePiInterface::readOdometer(uint32_t &timeStamp, int32_t &wheelCount, uint32_t &dt) {
  ioctl(fileno(bus),I2C_SLAVE,ODOMETER_ADDRESS);
  char buf[0x0C];
  buf[0]=0x00;
  fwrite(buf,1,1,bus);
  fread(buf,1,12,bus);
  wheelCount=readBuf_le<int32_t>(buf,0);
  dt        =readBuf_le<uint32_t>(buf,4);
  timeStamp =readBuf_le<uint32_t>(buf,8);
}

void HardwarePiInterface::readGyro(int g[]) {

}

HardwarePiInterface::HardwarePiInterface(Servo& Lsteering, Servo& Lthrottle):Interface(Lsteering,Lthrottle),t0(-1.0) {
  //Setup for GPIO (for buttons)
  wiringPiSetupGpio();
//  ppsf=fopen("/dev/pps0","r");
//  time_pps_create(fileno(ppsf), &pps);
  //Open the I2C bus
  bus=fopen("/dev/i2c-1","w");
  //Turn off buffering, so that the data actually flows in a timely manner
  setbuf(bus,nullptr);
  if(bus==nullptr) printf("Couldn't open bus: errno %d",errno);

  //Initialize the MPU9250
  mpu.begin(bus,0,0);
//  int gps=open("/dev/ttyAMA0",O_RDONLY | O_NONBLOCK);
//  gpsf=fdopen(gps,"rb");
}

HardwarePiInterface::~HardwarePiInterface() {
//  time_pps_destroy(pps);
//  fclose(ppsf);
//  fclose(bus);
}

HardwarePiInterfaceBlaster::HardwarePiInterfaceBlaster():hardSteering(0),hardThrottle(4),HardwarePiInterface(hardSteering,hardThrottle) {
  blaster=fopen("/dev/servoblaster","w");
  hardSteering.begin(blaster);
  hardThrottle.begin(blaster);
};

HardwarePiInterfaceBlaster::~HardwarePiInterfaceBlaster() {
  fclose(blaster);
}

HardwarePiInterfaceArduino::HardwarePiInterfaceArduino():hardSteering(0),hardThrottle(1),HardwarePiInterface(hardSteering,hardThrottle) {
  hardSteering.begin(bus);
  hardThrottle.begin(bus);
};

HardwarePiInterfaceArduino::~HardwarePiInterfaceArduino() {

}


