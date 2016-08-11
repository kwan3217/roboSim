#ifndef HardwarePi_h
#define HardwarePi_h

#include "robot.h"
#include <sys/timepps.h>
#include <cstdio>

/** Physical servo interface, via the /dev/servoblaster driver
 *
 */
class HardwarePiServoBlaster: public Servo {
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
class HardwarePiServoArduino: public Servo {
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
  static const int GYROSCOPE_ADDRESS=0x1E;///< 7-bit I2C address of MPU9250 used as gyrocompass
  char gpsBuf[128]; ///< GPS data buffer
  FILE* gpsf; ///<GPS NMEA stream
  int gpsLen; ///<Number of bytes in the GPS buffer
  int gpsPtr; ///<Index of next byte to be read
  void fillGpsBuf();
protected:
  FILE* bus; ///< I2C bus stream
public:
  virtual double checkPPS();
  virtual bool checkNavChar(); 
  virtual char readChar();
  virtual double time();
  virtual void readOdometer(uint32_t &timeStamp, int32_t &wheelCount, uint32_t &dt);
  virtual void readGyro(int g[]);
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
  FILE* bus; ///< Stream for I2C bus. This will be passed to both servos
  HardwarePiServoArduino hardSteering; ///< Steering servo interface
  HardwarePiServoArduino hardThrottle; ///< Throttle servo interface
public:
  HardwarePiInterfaceArduino();
  virtual ~HardwarePiInterfaceArduino();
};

#endif
