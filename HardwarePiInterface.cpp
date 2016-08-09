#include <time.h>
#include "HardwarePiInterface.h"
#include <linux/i2c-dev.h>
#include <fcntl.h>

void HardwarePiServoBlaster::write(int n) {
  fprintf(ouf,"%d=%d\n",channel,n);
  fflush(ouf);
}

void HardwarePiServoArduino::write(int n) {
  ioctl(fileno(bus),I2C_SLAVE,ADDRESS);
  char buf[3];
  buf[0]=0x2C+2*channel;
  buf[1]=(n >> 0) & 0xFF;
  buf[2]=(n >> 8) & 0xFF;
  fwrite(buf,1,3,bus);
}


/**
 * @copydoc Interface::checkPPS()
 * \internal
 * Implemented using the LinuxPPS driver, which in turn implements RFC2783, Pulse-Per-Second API. If the interface epoch isn't valid
 * at the first PPS, then the epoch is set to the time of the PPS.
 *
 */
double HardwarePiInterface::checkPPS() {
  pps_info_t info;
  static const struct timespec timeout={0,0};
  time_pps_fetch(pps,PPS_TSFMT_TSPEC,&info,&timeout);
  double t=ts2t(info.assert_timestamp);
  if(t0<t) t0=t;
  return t-t0;
}

/** Makes sure that GPS buffer has data to read. If there is still data in the buffer that hasn't been spooled out, return immediately. If
 * we have read to the end of the buffer, then try to read a full buffer, but set the length of data left to the amount we actually read.
 * The file has been opened with O_NONBLOCK so it should return immediately even if there isn't a full buffer worth of data to read.
 */
void HardwarePiInterface::fillGpsBuf() {
  if(gpsPtr<gpsLen) return;
  gpsLen=fread(gpsBuf,1,sizeof(gpsBuf),gpsf);
  gpsPtr=0;
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

static inline uint32_t buf_uint32_le(char buf[], int ofs) {
  return ((uint32_t(buf[ofs+0]) & 0xFF)<< 0) |
		 ((uint32_t(buf[ofs+1]) & 0xFF)<< 8) |
		 ((uint32_t(buf[ofs+2]) & 0xFF)<<16) |
	     ((uint32_t(buf[ofs+3]) & 0xFF)<<24);
}

static inline int32_t buf_int32_le(char buf[], int ofs) {
  return (( int32_t(buf[ofs+0]) & 0xFF)<< 0) |
		 (( int32_t(buf[ofs+1]) & 0xFF)<< 8) |
		 (( int32_t(buf[ofs+2]) & 0xFF)<<16) |
	     (( int32_t(buf[ofs+3]) & 0xFF)<<24);
}

static inline uint32_t buf_uint32_be(char buf[], int ofs) {
  return ((uint32_t(buf[ofs+0]) & 0xFF)<<24) |
		 ((uint32_t(buf[ofs+1]) & 0xFF)<<16) |
		 ((uint32_t(buf[ofs+2]) & 0xFF)<< 8) |
	     ((uint32_t(buf[ofs+3]) & 0xFF)<< 0);
}

static inline int32_t buf_int32_be(char buf[], int ofs) {
  return ( (int32_t(buf[ofs+0]) & 0xFF)<<24) |
		 ( (int32_t(buf[ofs+1]) & 0xFF)<<16) |
		 ( (int32_t(buf[ofs+2]) & 0xFF)<< 8) |
	     ( (int32_t(buf[ofs+3]) & 0xFF)<< 0);
}

static inline uint16_t buf_uint16_le(char buf[], int ofs) {
  return ((uint16_t(buf[ofs+0]) & 0xFF)<< 0) |
		 ((uint16_t(buf[ofs+1]) & 0xFF)<< 8) ;
}

static inline int16_t buf_int16_le(char buf[], int ofs) {
  return (( int16_t(buf[ofs+0]) & 0xFF)<< 0) |
		 (( int16_t(buf[ofs+1]) & 0xFF)<< 8) ;
}

static inline uint16_t buf_uint16_be(char buf[], int ofs) {
  return ((uint16_t(buf[ofs+0]) & 0xFF)<< 8) |
		 ((uint16_t(buf[ofs+1]) & 0xFF)<< 0) ;
}

static inline int16_t buf_int16_be(char buf[], int ofs) {
  return (( int16_t(buf[ofs+0]) & 0xFF)<< 8) |
		 (( int16_t(buf[ofs+1]) & 0xFF)<< 0) ;
}

void HardwarePiInterface::readOdometer(uint32_t &timeStamp, int32_t &wheelCount, uint32_t &dt) {
  ioctl(fileno(bus),I2C_SLAVE,ODOMETER_ADDRESS);
  char buf[0x0C];
  buf[0]=0x00;
  fwrite(buf,1,1,bus);
  fread(buf,1,12,bus);
  wheelCount=buf_int32_le(buf,0);
  dt        =buf_uint32_le(buf,4);
  timeStamp =buf_uint32_le(buf,8);
}

void HardwarePiInterface::readGyro(int g[]) {
  char buf[6];
  ioctl(fileno(bus),I2C_SLAVE,GYROSCOPE_ADDRESS);
  buf[0]=0x43;
  fwrite(buf,1,1,bus);
  fread(buf,1,6,bus);
  g[0]=buf_int16_be(buf,0);
  g[1]=buf_int16_be(buf,2);
  g[2]=buf_int16_be(buf,4);
}

HardwarePiInterface::HardwarePiInterface(Servo& Lsteering, Servo& Lthrottle):Interface(Lsteering,Lthrottle),t0(-1.0) {
  ppsf=fopen("/dev/pps0","r");
  time_pps_create(fileno(ppsf), &pps);
  bus=fopen("/dev/i2c-1","w");
  int gps=open("/dev/ttyAMA0",O_RDONLY | O_NONBLOCK);
  gpsf=fdopen(gps,"rb");
}

HardwarePiInterface::~HardwarePiInterface() {
  time_pps_destroy(pps);
  fclose(ppsf);
  fclose(bus);
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


