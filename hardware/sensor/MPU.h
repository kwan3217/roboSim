#ifndef MPU_h
#define MPU_h
#include <inttypes.h>
#include "I2C.h"

/** Driver for MPU6xx0 series and 9xx0 series motion processing units
 *
 */
class MPU {
protected:
  /** Write a byte register
   \param addr register address
   \param val value to write
   \return true if the write worked, false otherwise
   */
  virtual bool write(uint8_t addr, uint8_t val)=0;
  /** Read a byte register
   \param[in] addr register address
   \return value that was read
   \param[out] success true if the read worked, false otherwise
   */
  virtual uint8_t read(uint8_t addr, bool& success)=0;
  /** Read a 16-bit register. Registers on the MPU series which are 16-bit are invariably two's complement signed in big endian order.
   \param[in] addr register address
   \return value that was read
   \param[out] success true if the read worked, false otherwise
   */
  virtual int16_t read16(uint8_t addr, bool& success)=0;
  /** Read a series of registers in burst mode
   \param[in] addr register address to start at
   \param[out] buf buffer to read into, must contain space for at leaste len bytes
   \param[in] len
   \return true if the read worked, false otherwise
   */
  virtual bool read(uint8_t addr, char buf[], int len)=0;
  //Register addresses. In decimal because the majority of the register map
  //document uses decimal registers
  static const int SMPLRT_DIV        = 25; ///< Register address for this register
  static const int CONFIG            = 26; ///< Register address for this register
  static const int GYRO_CONFIG       = 27; ///< Register address for this register
  static const int ACCEL_CONFIG      = 28; ///< Register address for this register
  static const int INT_PIN_CFG       = 55; ///< Register address for this register
  static const int ACCEL_XOUT_H      = 59; ///< Register address for this register
  static const int ACCEL_XOUT_L      = 60; ///< Register address for this register
  static const int ACCEL_YOUT_H      = 61; ///< Register address for this register
  static const int ACCEL_YOUT_L      = 62; ///< Register address for this register
  static const int ACCEL_ZOUT_H      = 63; ///< Register address for this register
  static const int ACCEL_ZOUT_L      = 64; ///< Register address for this register
  static const int TEMP_OUT_H        = 65; ///< Register address for this register
  static const int TEMP_OUT_L        = 66; ///< Register address for this register
  static const int GYRO_XOUT_H       = 67; ///< Register address for this register
  static const int SIGNAL_PATH_RESET =104; ///< Register address for this register
  static const int USER_CTRL         =106; ///< Register address for this register
  static const int PWR_MGMT_1        =107; ///< Register address for this register
  static const int PWR_MGMT_2        =108; ///< Register address for this register
  static const int WHOAMI            =117; ///< Register address for this register
public:
  bool begin();
  unsigned char whoami(bool& success) {return read(WHOAMI,success);};
  bool configure(uint8_t gyro_scale, uint8_t acc_scale, uint8_t bandwidth, uint8_t sample_rate);
  bool readConfig(char buf[]);
  /** Perform a gyro readout in burst mode
   * @param[out] gx rotation rate around the x axis in DN
   * @param[out] gy rotation rate around the y axis in DN
   * @param[out] gz rotation rate around the z axis in DN
   * @return true if read worked, false if not
   */
  bool readGyro(int16_t& gx, int16_t& gy, int16_t& gz);
  /** Perform a gyro readout in burst mode
   * @param[out] gx rotation rate around the x axis in DN
   * @param[out] gy rotation rate around the y axis in DN
   * @param[out] gz rotation rate around the z axis in DN
   * @param[out] t  temperature in DN
   * @return true if read worked, false if not
   */
  bool readGyro(int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t);
  /** Perform a accelerometer readout in burst mode
   * @param[out] ax acceleration along the x axis in DN
   * @param[out] ay acceleration along the y axis in DN
   * @param[out] az acceleration along the z axis in DN
   * @return true if read worked, false if not
   */
  bool readAcc(int16_t& ax, int16_t& ay, int16_t& az);
  /** Perform a accelerometer readout in burst mode
   * @param[out] ax acceleration along the x axis in DN
   * @param[out] ay acceleration along the y axis in DN
   * @param[out] az acceleration along the z axis in DN
   * @param[out] t  temperature in DN
   * @return true if read worked, false if not
   */
  bool readAcc(int16_t& ax, int16_t& ay, int16_t& az, int16_t& t);
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
  bool readMPU(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t);
};

class MPUI2C: public MPU {
private:
  I2C_t bus;
  virtual bool write(uint8_t addr, uint8_t val) {return writeI2Creg(bus, ADDRESS, addr, val);};
  virtual uint8_t read(uint8_t addr, bool& success) {return readI2Creg(bus, ADDRESS, addr, success);};
  virtual int16_t read16(uint8_t addr, bool& success) {return readI2Creg_be<int16_t>(bus, ADDRESS, addr, success);};
  virtual bool read(uint8_t addr, char buf[], int len) {return readI2Creg(bus,ADDRESS,addr,buf,len);};
public:
  static const int ADDRESS=0x68;///< 7-bit I2C address of MPU9250 used as gyrocompass
  bool begin(I2C_t Lbus) {
    bus=Lbus;
    return MPU::begin();
  }
};

/** Driver for AK8963 magnetometer, embedded in the MPU9250. Electrically it is a separate device,
 *  connected to an I2C bus inside the MPU. This internal bus can be configured to connect directly to
 *  the main bus the rest of the MPU is on. The MPU must be configured like this first in order for the
 *  AK part to even be visible to the host.
 *
 */
class AK89xx {
private:
  I2C_t bus;
  /** Write a byte register
   \param addr register address
   \param val value to write
   \return true if the write worked, false otherwise
   */
  virtual bool write(uint8_t addr, uint8_t val) {return writeI2Creg(bus, ADDRESS, addr, val);};
  /** Read a byte register
   \param[in] addr register address
   \return value that was read
   \param[out] success true if the read worked, false otherwise
   */
  virtual uint8_t read(uint8_t addr, bool& success) {return readI2Creg(bus, ADDRESS, addr, success);};
  /** Read a 16-bit register. Registers on the MPU series which are 16-bit are invariably two's complement signed in big endian order.
   \param[in] addr register address
   \return value that was read
   \param[out] success true if the read worked, false otherwise
   */
  virtual int16_t read16(uint8_t addr, bool& success) {return readI2Creg_be<int16_t>(bus, ADDRESS, addr, success);};
  /** Read a series of registers in burst mode
   \param[in] addr register address to start at
   \param[out] buf buffer to read into, must contain space for at leaste len bytes
   \param[in] len
   \return true if the read worked, false otherwise
   */
  virtual bool read(uint8_t addr, char buf[], int len) {return readI2Creg(bus,ADDRESS,addr,buf,len);};
  //Register addresses. In hex because the magnetometer part of the register map
  //document uses hex registers
  static const int WIA    = 0x00; ///< Register address for register Device ID (RO)
  static const int INFO   = 0x01; ///< Register address for register Information (RO)
  static const int ST1    = 0x02; ///< Register address for register Status 1 (RO)
  static const int HXL    = 0x03; ///< Register address for register X measurement low byte (RO)
  static const int HXH    = 0x04; ///< Register address for register X measurement high byte (RO)
  static const int HYL    = 0x05; ///< Register address for register Y measurement low byte (RO)
  static const int HYH    = 0x06; ///< Register address for register Y measurement high byte (RO)
  static const int HZL    = 0x07; ///< Register address for register Z measurement low byte (RO)
  static const int HZH    = 0x08; ///< Register address for register Z measurement high byte (RO)
  static const int ST2    = 0x09; ///< Register address for register Status 2 (RO)
  static const int CNTL1  = 0x0A; ///< Register address for register CNTL1 (RW)
  static const int CNTL2  = 0x0B; ///< Register address for register CNTL2 (RW)
  static const int ASTC   = 0x0C; ///< Register address for register Self-test (RW)
  static const int TS1    = 0x0D; ///< Register address for register Test 1 (RW, but do not access)
  static const int TS2    = 0x0E; ///< Register address for register Test 2 (RW, but do not access)
  static const int I2CDIS = 0x0F; ///< Register address for register I2C disable (RW)
  static const int ASAX   = 0x10; ///< Register address for register X-axis sensitivity adjustment value (RO, Fuse ROM)
  static const int ASAY   = 0x11; ///< Register address for register Y-axis sensitivity adjustment value (RO, Fuse ROM)
  static const int ASAZ   = 0x12; ///< Register address for register Z-axis sensitivity adjustment value (RO, Fuse ROM)
  static const int RSV    = 0x13; ///< Register address for register Reserved (RO, but do not access)
public:
  bool begin();
  unsigned char whoami(bool& success) {return read(WIA,success);};
  bool configure(uint8_t mode=0x02, bool bit16=true);
  bool readConfig(char buf[]);
  /** Perform a magnetometer readout in burst mode
   * @param[out] bx measurement on x axis in DN
   * @param[out] by measurement on y axis in DN
   * @param[out] bz measurement on z axis in DN
   * @return true if read worked, false if not
   */
  bool read(int16_t& gx, int16_t& gy, int16_t& gz);
  static const int ADDRESS=0x0C;///< 7-bit I2C address of AX89xx
  bool begin(I2C_t Lbus) {
    bus=Lbus;
    return begin();
  }
};

/** Driver for MPU9250, which is accesed by I2C and includes an AK8963 magnetometer. As it happens,
 * the register map for the MPU9150 is close enough to the same, and the map of the AK8975 (embedded
 * in a 9150) is close enough to the same, that this driver should work for an MPU9150 with no changes.
 */
class MPU9250: public MPUI2C {
public:
  AK89xx ak;
  bool begin(I2C_t Lbus) {
    if(!MPUI2C::begin(Lbus)) return false;
    return ak.begin(Lbus);
  }
};

#endif
