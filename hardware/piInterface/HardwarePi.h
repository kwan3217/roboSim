#ifndef HardwarePi_h
#define HardwarePi_h
/** @file */

#include "Interface.h"
#include <sys/timepps.h>
#include <cstdio> //used for FILE*
#include "buffer.h"
#include "I2C.h"
#include "sensor/MPU.h"

class HardwarePiServo: public Servo {
public:
  virtual void begin(I2C_t)=0;
};

/** Physical servo interface, via the Arduino which is also hosting the odometer
 *
 */
class HardwarePiServoArduino: public HardwarePiServo {
private:
  int channel; ///<Channel number to write, either 0 or 1
  I2C_t bus; ///<I2C bus stream, opened somewhere else because other devices are on the bus
  static const int ADDRESS=0x55; ///< 7-bit address of Arduino
public:
  virtual void write(int n);
  virtual void begin(I2C_t Lbus) {bus=Lbus;};
  HardwarePiServoArduino(int Lchannel):channel(Lchannel),bus(-1) {};
  HardwarePiServoArduino(I2C_t Lbus, int Lchannel):HardwarePiServoArduino(Lchannel) {begin(Lbus);};
  ~HardwarePiServoArduino() {};
};

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

/** Hardware interface using a Raspberry Pi, as currently intended for Yukari 4
 *
 */
class HardwarePiInterface: public Interface {
private:
  struct timespec t0,last_pps; ///<Epoch time in Unix seconds, so we don't have to deal with an epoch over a billion seconds ago
  FILE* ppsf; ///< PPS stream
  pps_handle_t pps; ///<PPS instance handle
  /** Convert a struct timespec into a count of seconds. The integer part of the return will be specified by tv_sec, while the fractional
   * part is specified by tv_nsec.
   * @param ts a struct timespec to convert
   * @return equivalent timestamp in double precision count of seconds.
   */
  static fp ts2t(struct timespec ts) {return ts.tv_sec+fp(ts.tv_nsec)/1'000'000'000.0;};
  static const int ODOMETER_ADDRESS=0x55; ///< 7-bit I2C address of Arduino used as odometer
  char gpsBuf[128]; ///< GPS data buffer
  FILE* gpsf; ///<GPS NMEA stream
  int gpsLen; ///<Number of bytes in the GPS buffer
  int gpsPtr; ///<Index of next byte to be read
  void fillGpsBuf();
protected:
public:
  I2C_t bus; ///< I2C bus stream
  MPU9250 mpu;
  struct timespec get_raw_t0() {return t0;};
  struct timespec get_raw_pps() {return last_pps;};
  virtual bool checkPPS(fp& t);
  virtual bool checkNavChar();
  virtual char readChar();
  virtual fp time();
  struct timespec epoch() {return t0;};
  virtual void readOdometer(uint32_t &timeStamp, int32_t &wheelCount, uint32_t &dt);
  virtual bool readAcc(int16_t a[]);
  virtual bool readAcc(int16_t a[], int16_t& t);
  virtual bool readMag(int16_t b[]);
  virtual bool readGyro(int16_t g[]);
  virtual bool readGyro(int16_t g[], int16_t& t);
  virtual bool readMPU(int16_t a[], int16_t g[], int16_t& t);
  virtual bool button(int pin=17);
  HardwarePiInterface(Servo& steering, Servo& throttle);
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
