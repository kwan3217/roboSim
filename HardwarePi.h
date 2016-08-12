#ifndef HardwarePi_h
#define HardwarePi_h

#include "robot.h"
#include <sys/timepps.h>
#include <cstdio>
#include <linux/i2c-dev.h>
#include <fcntl.h>

class HardwarePiServo: public Servo {
public:
  virtual void begin(FILE*)=0;
};

/** Physical servo interface, via the /dev/servoblaster driver
 *
 */
class HardwarePiServoBlaster: public HardwarePiServo {
  FILE* ouf; ///< ServoBlaster device stream, opened somewhere else because multiple servos will be using the same file
  int channel; ///<Channel number to write, in this case the servo channel number defined by ServoBlaster
public:
  virtual void write(int n);
  virtual void begin(FILE* Louf) {ouf=Louf;};
  HardwarePiServoBlaster(int Lchannel):channel(Lchannel),ouf(nullptr) {};
  HardwarePiServoBlaster(FILE* Louf, int Lchannel):HardwarePiServoBlaster(Lchannel) {begin(Louf);};
  ~HardwarePiServoBlaster() {};
};

/** Physical servo interface, via the Arduino which is also hosting the odometer
 *
 */
class HardwarePiServoArduino: public HardwarePiServo {
private:
  FILE* bus; ///<I2C bus stream, opened somewhere else because other devices are on the bus
  static const int ADDRESS=0x55; ///< 7-bit address of Arduino
  int channel; ///<Channel number to write, either 0 or 1
public:
  virtual void write(int n);
  virtual void begin(FILE* Lbus) {bus=Lbus;};
  HardwarePiServoArduino(int Lchannel):channel(Lchannel),bus(nullptr) {};
  HardwarePiServoArduino(FILE* Lbus, int Lchannel):HardwarePiServoArduino(Lchannel) {begin(Lbus);};
  ~HardwarePiServoArduino() {};
};

/** Driver for MPU6xx0 series and 9xx0 series motion processing units. Doesn't init the
 *
 */
class MPU {
protected:
  virtual void write(uint8_t addr, uint8_t val)=0;
  virtual uint8_t read(uint8_t addr)=0;
  virtual int16_t read16(uint8_t addr)=0;
public:
  unsigned char whoami() {return read(0x75);};
  /** Perform a gyro readout in burst mode
   * @param[out] gx rotation rate around the x axis in DN
   * @param[out] gy rotation rate around the y axis in DN
   * @param[out] gz rotation rate around the z axis in DN
   */
  virtual bool readGyro(int16_t& gx, int16_t& gy, int16_t& gz)=0;
  /** Perform a accelerometer readout in burst mode
   * @param[out] ax acceleration along the x axis in DN
   * @param[out] ay acceleration along the y axis in DN
   * @param[out] az acceleration along the z axis in DN
   */
  virtual bool readAcc(int16_t& ax, int16_t& ay, int16_t& az)=0;
  /** Perform an accelerometer and gyro readout in burst mode
   * @param[out] ax acceleration along the x axis in DN
   * @param[out] ay acceleration along the y axis in DN
   * @param[out] az acceleration along the z axis in DN
   * @param[out] gx rotation rate around the x axis in DN
   * @param[out] gy rotation rate around the y axis in DN
   * @param[out] gz rotation rate around the z axis in DN
   * @param[out] t temperature in DN
   */
  virtual bool read(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t)=0;
  void begin(uint8_t gyro_scale, uint8_t acc_scale, uint8_t bandwidth=0, uint8_t sample_rate=0);
};

class MPUI2C: public MPU {
private:
  FILE* bus;
  static const int ADDRESS=0x68;///< 7-bit I2C address of MPU9250 used as gyrocompass
  virtual void write(uint8_t addr, uint8_t val);
  virtual uint8_t read(uint8_t addr);
  virtual int16_t read16(uint8_t addr);
public:
  virtual bool readGyro(int16_t& gx, int16_t& gy, int16_t& gz);
  virtual bool readAcc(int16_t& ax, int16_t& ay, int16_t& az);
  virtual bool read(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t);
  void begin(FILE* Lbus, uint8_t gyro_scale, uint8_t acc_scale, uint8_t bandwidth=0, uint8_t sample_rate=0) {
	bus=Lbus;
    MPU::begin(gyro_scale,acc_scale,bandwidth,sample_rate);
  }
};

/** Hardware interface using a Raspberry Pi, as currently intended for Yukari 4
 *
 */
class HardwarePiInterface: public Interface {
private:
  double t0; ///<Epoch time in Unix seconds, so we don't have to deal with an epoch over a billion seconds ago
  FILE* ppsf; ///< PPS stream
  pps_handle_t pps; ///<PPS instance handle
  /** Convert a struct timespec into a count of seconds. The integer part of the return will be specified by tv_sec, while the fractional
   * part is specified by tv_nsec.
   * @param ts a struct timespec to convert
   * @return equivalent timestamp in double precision count of seconds.
   */
  static double ts2t(struct timespec ts) {return ts.tv_sec+double(ts.tv_nsec)/1'000'000'000.0;};
  static const int ODOMETER_ADDRESS=0x55; ///< 7-bit I2C address of Arduino used as odometer
  char gpsBuf[128]; ///< GPS data buffer
  FILE* gpsf; ///<GPS NMEA stream
  int gpsLen; ///<Number of bytes in the GPS buffer
  int gpsPtr; ///<Index of next byte to be read
  void fillGpsBuf();
  MPUI2C mpu;
protected:
  FILE* bus; ///< I2C bus stream
public:
  virtual double checkPPS();
  virtual bool checkNavChar();
  virtual char readChar();
  virtual double time();
  virtual void readOdometer(uint32_t &timeStamp, int32_t &wheelCount, uint32_t &dt);
  virtual void readGyro(int g[]);
  virtual bool button(int pin=17);
  HardwarePiInterface(Servo& Lsteering, Servo& Lthrottle);
  virtual ~HardwarePiInterface();
};

/** Hardware interface for Raspberry Pi, using the ServoBlaster servo interface
 *
 */
class HardwarePiInterfaceBlaster: public HardwarePiInterface {
private:
  FILE* blaster; ///< Stream for /dev/servoblaster. This will be passed to both servos
  HardwarePiServoBlaster hardSteering; ///< Steering servo interface
  HardwarePiServoBlaster hardThrottle; ///< Throttle servo interface
public:
  HardwarePiInterfaceBlaster();
  virtual ~HardwarePiInterfaceBlaster();
};

/** Hardware interface for Raspberry Pi, using the Arduino servo interface
 *
 */
class HardwarePiInterfaceArduino: public HardwarePiInterface {
private:
  HardwarePiServoArduino hardSteering; ///< Steering servo interface
  HardwarePiServoArduino hardThrottle; ///< Throttle servo interface
public:
  HardwarePiInterfaceArduino();
  virtual ~HardwarePiInterfaceArduino();
};

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

static inline int writeI2C(FILE* bus, int addr, char* buf, int len) {
  ioctl(fileno(bus),I2C_SLAVE,addr);
  return fwrite(buf,1,len,bus);
}

static inline int writeI2Creg(FILE* bus, int slaveaddr, int regaddr, uint8_t data) {
  char buf[2];
  buf[0]=regaddr;
  buf[1]=data;
  return writeI2C(bus,slaveaddr,buf,2);
}

static inline int writeI2Creg_le(FILE* bus, int slaveaddr, int regaddr, uint16_t data) {
  char buf[3];
  buf[0]=regaddr;
  buf[1]=(data >> 0) & 0xFF;
  buf[2]=(data >> 8) & 0xFF;
  return writeI2C(bus,slaveaddr,buf,sizeof(buf));
}

static inline int writeI2Creg_be(FILE* bus, int slaveaddr, int regaddr, uint16_t data) {
  char buf[3];
  buf[0]=regaddr;
  buf[1]=(data >> 8) & 0xFF;
  buf[2]=(data >> 0) & 0xFF;
  return writeI2C(bus,slaveaddr,buf,sizeof(buf));
}

static inline int writeI2Creg_le(FILE* bus, int slaveaddr, int regaddr, uint32_t data) {
  char buf[5];
  buf[0]=regaddr;
  buf[1]=(data >> 0) & 0xFF;
  buf[2]=(data >> 8) & 0xFF;
  buf[3]=(data >>16) & 0xFF;
  buf[4]=(data >>24) & 0xFF;
  return writeI2C(bus,slaveaddr,buf,sizeof(buf));
}

static inline int writeI2Creg_be(FILE* bus, int slaveaddr, int regaddr, uint32_t data) {
  char buf[5];
  buf[0]=regaddr;
  buf[1]=(data >>24) & 0xFF;
  buf[2]=(data >>16) & 0xFF;
  buf[3]=(data >> 8) & 0xFF;
  buf[4]=(data >> 0) & 0xFF;
  return writeI2C(bus,slaveaddr,buf,sizeof(buf));
}

static inline int writeI2Creg(FILE* bus, int slaveaddr, int regaddr, int8_t data) {
  return writeI2Creg(bus,slaveaddr,regaddr,(uint8_t)data);
}

static inline int writeI2Creg_le(FILE* bus, int slaveaddr, int regaddr, int16_t data) {
  return writeI2Creg_le(bus,slaveaddr,regaddr,(uint16_t)data);
}

static inline int writeI2Creg_be(FILE* bus, int slaveaddr, int regaddr, int16_t data) {
  return writeI2Creg_be(bus,slaveaddr,regaddr,(uint16_t)data);
}

static inline int writeI2Creg_le(FILE* bus, int slaveaddr, int regaddr, int32_t data) {
  return writeI2Creg_le(bus,slaveaddr,regaddr,(uint32_t)data);
}

static inline int writeI2Creg_be(FILE* bus, int slaveaddr, int regaddr, int32_t data) {
  return writeI2Creg_be(bus,slaveaddr,regaddr,(uint32_t)data);
}

#endif
