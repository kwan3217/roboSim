#ifndef HardwarePi_h
#define HardwarePi_h
/** @file */

#include "robot.h"
#include <sys/timepps.h>
#include <cstdio>
#include "buffer.h"
#include "I2C.h"

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
  virtual bool write(uint8_t addr, uint8_t val)=0;
  virtual uint8_t read(uint8_t addr, bool& success)=0;
  virtual int16_t read16(uint8_t addr, bool& success)=0;
public:
  unsigned char whoami(bool& success) {return read(0x75,success);};
  /** Perform a gyro readout in burst mode
   * @param[out] gx rotation rate around the x axis in DN
   * @param[out] gy rotation rate around the y axis in DN
   * @param[out] gz rotation rate around the z axis in DN
   * @return true if read worked, false if not
   */
  virtual bool readGyro(int16_t& gx, int16_t& gy, int16_t& gz)=0;
  /** Perform a accelerometer readout in burst mode
   * @param[out] ax acceleration along the x axis in DN
   * @param[out] ay acceleration along the y axis in DN
   * @param[out] az acceleration along the z axis in DN
   * @return true if read worked, false if not
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
   * @return true if read worked, false if not
   */
  virtual bool read(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t)=0;
  void begin(uint8_t gyro_scale, uint8_t acc_scale, uint8_t bandwidth, uint8_t sample_rate);
};

class MPUI2C: public MPU {
private:
  I2C_t bus;
  virtual bool write(uint8_t addr, uint8_t val) {return writeI2Creg(bus, ADDRESS, addr, val);};
  virtual uint8_t read(uint8_t addr, bool& success) {return readI2Creg(bus, ADDRESS, addr, success);};
  virtual int16_t read16(uint8_t addr, bool& success) {return readI2Creg_be<int16_t>(bus, ADDRESS, addr, success);};
public:
  static const int ADDRESS=0x68;///< 7-bit I2C address of MPU9250 used as gyrocompass
  bool readConfig(char buf[]);
  virtual bool readGyro(int16_t& gx, int16_t& gy, int16_t& gz);
  virtual bool readAcc(int16_t& ax, int16_t& ay, int16_t& az);
  virtual bool read(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t);
  void begin(I2C_t Lbus, uint8_t gyro_scale, uint8_t acc_scale, uint8_t bandwidth, uint8_t sample_rate) {
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
public:
  I2C_t bus; ///< I2C bus stream
  MPUI2C mpu;
  virtual double checkPPS();
  virtual bool checkNavChar();
  virtual char readChar();
  virtual double time();
  virtual void readOdometer(uint32_t &timeStamp, int32_t &wheelCount, uint32_t &dt);
  virtual bool readGyro(int g[]);
  virtual bool button(int pin=17);
  HardwarePiInterface(Servo& Lsteering, Servo& Lthrottle, uint8_t bandwidth=0, uint8_t sample_rate=0);
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
