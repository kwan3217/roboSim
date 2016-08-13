#ifndef HardwarePi_h
#define HardwarePi_h
/** @file */

#include "robot.h"
#include <sys/timepps.h>
#include <cstdio>
#include <linux/i2c-dev.h>
#include <fcntl.h>

typedef int I2C_t;

/** Extract a little-endian integer from a byte buffer
 * \tparam T type of integer to extract
 * \param buf byte buffer
 * \param ofs offset of first byte in number (will go into least significant byte)
 */
template<typename T>
static inline T readBuf_le(char buf[], int ofs) {
  T result=0;
  for(int i=0;i<sizeof(T);i++) result |= (T(buf[ofs+i]) & 0xFF) << i*8;
  return result;
}

/** Extract a big-endian integer from a byte buffer
 * \tparam T type of integer to extract
 * \param buf byte buffer
 * \param ofs offset of first byte in number (will go into most significant byte)
 */
template<typename T>
static inline T readBuf_be(char buf[], int ofs) {
  T result=0;
  for(int i=0;i<sizeof(T);i++) result |= (T(buf[ofs+i]) & 0xFF) << (sizeof(T)-1-i)*8;
  return result;
}

/** Insert a little-endian integer into a byte buffer
 * \tparam T type of integer to insert
 * \param buf byte buffer
 * \param ofs offset of first byte in number (least significant byte will go here)
 */
template<typename T>
static inline void writeBuf_le(char buf[], int ofs, T data) {
  for(int i=0;i<sizeof(T);i++) buf[ofs+i] = char((data >> i*8) & 0xFF);
}

/** Insert a big-endian integer into a byte buffer
 * \tparam T type of integer to insert
 * \param buf byte buffer
 * \param ofs offset of first byte in number (most significant byte will go here)
 */
template<typename T>
static inline void writeBuf_be(char buf[], int ofs, T data) {
  for(int i=0;i<sizeof(T);i++) buf[ofs+i] = char((data >> (sizeof(T)-1-i)*8) & 0xFF);
}

/** Write a buffer to the I2C bus
 * \param bus stream representing I2C bus
 * \param addr slave address to write to
 * \param buf buffer to write
 * \param buf number of bytes to write
 */
static inline bool writeI2C(I2C_t bus,  uint8_t addr, char* buf, int len) {
  ioctl(bus,I2C_SLAVE,addr);
  bool result=(len==write(bus,buf,len));
  return result;
}

/** Read a buffer from the I2C bus
 * \param bus stream representing I2C bus
 * \param addr slave address to write to
 * \param buf buffer to read to
 * \param buf number of bytes to read
 */
static inline bool readI2C(I2C_t bus,  uint8_t addr, char* buf, int len) {
  ioctl(bus,I2C_SLAVE,addr);
  bool result=(len==read(bus,buf,len));
  return result;
}

/** Write a byte to a specified register in a slave on an I2C bus
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to write to
 * \param regaddr register address to write to
 * \param data value to write
 */
static inline bool writeI2Creg(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr, uint8_t data) {
  char buf[2];
  buf[0]=regaddr;
  buf[1]=data;
  bool result=writeI2C(bus,slaveaddr,buf,2);
  return result;
}

/** Write a value to a multi-byte little-endian register in a slave on an I2C bus
 * \tparam T integer type of register
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to write to
 * \param regaddr register address of first (least significant) byte to write to
 * \param data value to write
 */
template<typename T>
static inline int writeI2Creg_le(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr, T data) {
  char buf[sizeof(T)+1];
  buf[0]=regaddr;
  writeBuf_le<T>(buf,1,data);
  return writeI2C(bus,slaveaddr,buf,sizeof(T)+1);
}

/** Write a value to a multi-byte big-endian register in a slave on an I2C bus
 * \tparam T integer type of register
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to write to
 * \param regaddr register address of first (most significant) byte to write to
 * \param data value to write
 */
template<typename T>
static inline bool writeI2Creg_be(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr, T data) {
  char buf[sizeof(T)+1];
  buf[0]=regaddr;
  writeBuf_be<T>(buf,1,data);
  bool result=writeI2C(bus,slaveaddr,buf,sizeof(T)+1);
  return result;
}

/** Read a byte from a specified register in a slave on an I2C bus
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to read from
 * \param regaddr register address to read from
 * \return data value that was read
 */
static inline uint8_t readI2Creg(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr) {
  char buf=regaddr;
  if(!writeI2C(bus,slaveaddr,&buf,1)) printf("Addressing device failed, bus=%d slaveaddr=%02x\n",bus,slaveaddr);
  if(read(bus,&buf,1)!=1) printf("Reading device failed\n");
  return buf;
}

/** Read a series of consecutive byte registers in a slave on an I2C bus
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to read from
 * \param regaddr register address of first register to read from
 * \param buf buffer to read into
 * \param len number of bytes to read
 */
static inline bool readI2Creg(I2C_t bus, uint8_t slaveaddr, uint8_t regaddr, char* buf, int len) {
  buf[0]=regaddr;
  if(!writeI2C(bus,slaveaddr,buf,1)) printf("Addressing device failed\n");
  if(!readI2C(bus,slaveaddr,buf,len)) printf("Reading device failed\n");
}

/** Read a value from a multi-byte little-endian register in a slave on an I2C bus
 * \tparam T integer type of register
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to write to
 * \param regaddr register address of first (least significant) byte to write to
 * \return value in that multi-byte register
 */
template<typename T>
static inline T readI2Creg_le(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr) {
  char buf[sizeof(T)];
  readI2Creg(bus,slaveaddr,regaddr,buf,sizeof(T));
  return readBuf_le<T>(buf,0);
}

/** Read a value from a multi-byte big-endian register in a slave on an I2C bus
 * \tparam T integer type of register
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to write to
 * \param regaddr register address of first (most significant) byte to write to
 * \return value in that multi-byte register
 */
template<typename T>
static inline T readI2Creg_be(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr) {
  char buf[sizeof(T)];
  readI2Creg(bus,slaveaddr,regaddr,buf,sizeof(T));
  return readBuf_be<T>(buf,0);
}

class HardwarePiServo: public Servo {
public:
  virtual void begin(I2C_t)=0;
};

/** Physical servo interface, via the Arduino which is also hosting the odometer
 *
 */
class HardwarePiServoArduino: public HardwarePiServo {
private:
  I2C_t bus; ///<I2C bus stream, opened somewhere else because other devices are on the bus
  static const int ADDRESS=0x55; ///< 7-bit address of Arduino
  int channel; ///<Channel number to write, either 0 or 1
public:
  virtual void write(int n);
  virtual void begin(I2C_t Lbus) {bus=Lbus;};
  HardwarePiServoArduino(int Lchannel):channel(Lchannel),bus(-1) {};
  HardwarePiServoArduino(I2C_t Lbus, int Lchannel):HardwarePiServoArduino(Lchannel) {begin(Lbus);};
  ~HardwarePiServoArduino() {};
};

/** Driver for MPU6xx0 series and 9xx0 series motion processing units
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
  I2C_t bus;
  static const int ADDRESS=0x68;///< 7-bit I2C address of MPU9250 used as gyrocompass
  virtual void write(uint8_t addr, uint8_t val) {writeI2Creg(bus, ADDRESS, addr, val);};
  virtual uint8_t read(uint8_t addr) {return readI2Creg(bus, ADDRESS, addr);};
  virtual int16_t read16(uint8_t addr) {return readI2Creg_be<int16_t>(bus, ADDRESS, addr);};
public:
  virtual bool readGyro(int16_t& gx, int16_t& gy, int16_t& gz);
  virtual bool readAcc(int16_t& ax, int16_t& ay, int16_t& az);
  virtual bool read(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t);
  void begin(I2C_t Lbus, uint8_t gyro_scale, uint8_t acc_scale, uint8_t bandwidth=0, uint8_t sample_rate=0) {
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
protected:
  I2C_t bus; ///< I2C bus stream
public:
  MPUI2C mpu;
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

#endif
