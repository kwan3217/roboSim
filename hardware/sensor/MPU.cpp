#include "MPU.h"

#include "buffer.h" //for readbuf_be
#include "I2C.h"    //for I2C register access


/** Start the MPU9250 and initialize the configuration. Do anything necessary to init the part. Bus is available at this point.
*/
bool MPU::begin() {
  bool success;
  uint8_t who=whoami(success);
  if(!success||who!=0x71) {
//    printf("Failed: %02x=whoami(%s)",who,success?"true":"false");
    return false;
  }
  if(!write(PWR_MGMT_1, (1 << 7) | // 0 - reset everything, bit will auto-clear
                  (0 << 6) | // 0 - don't sleep
                  (0 << 5) | // 0 - don't cycle and sleep
                  (0b001 << 0)))  //clockselect 1 - auto-select best available source (used to be use X gyro)
  {return false;}
  usleep(50000); //Wait for this to go out
  //Set up I2C passthrough, so that the magnetic sensor on a 9150 can be accessed. Also set int levels
  if(!write(INT_PIN_CFG, (0 << 7) | //0 - INT line is active high
                         (0 << 6) | //0 - INT line is push-pull (no pullup needed, can't attach another int line and do a wired and)
                         (0 << 5) | //0 - LATCH_INT_EN is disabled, int line emits a 50us pulse. This is detectable by the capture line but doesn't need any action to reset
                         (1 << 4) | //1 - INT_RD_CLEAR on any read. If you want int status, read that register first on receiving an int
                         (0 << 3) | //0 - Frame Sync input - designed to trigger a read in sync with an external event, like a video frame capture. Not used in any Kwan design yet
                         (0 << 2) | //0 - Frame sync interrupt enabled - when set, a pulse on the FSYNC input to the 60x0 will cause an interrupt on the INT line to the host
                         (1 << 1))) //1 - I2C_BYPASS_EN - bypass enable. When set to 1, the auxillary bus (including the mag sensor on the 9150) is just tied to the main
                                    //                    bus so that the host can see the auxillary sensor(s) directly.);
  {return false;}
  //In order for passthrough to work, the 9150 I2C bus master must be disabled, but this is default, so we don't do anything with it
  return true;
}

/** Configure the MPU9250
 \param gyro_scale Gyro full scale. Physical value is +-250*2^gyro_scale deg/s, IE 250,500,1000,2000deg/s
 \param acc_scale Accelerometer full scale. Physical value is +-2*2^acc_scale g, IE 2,4,8,16g
 \param bandwidth Gyroscope bandwidth, gets written in DLPF_CFG field of CONFIG register (0x1a)
   selected from this table:
   bandwidth | filter bandwidth (Hz) | delay (ms) | internal read rate (kHz)
  ---------- | --------------------- | ---------- | -----------------------
       0     |    250                |   0.97     |  8
       1     |    184                |   2.90     |  1
       2     |     92                |   3.90     |  1
       3     |     41                |   5.90     |  1
       4     |     20                |   9.90     |  1
       5     |     10                |  17.85     |  1
       6     |      5                |  33.48     |  1
       7     |   3600                |   0.17     |  1
 \param sample_rate Sample rate divisor. Given the internal read rate, the MPU reports and can generate
        an interrupt every sample_rate+1 times the sensor is read internally. So, if the rate is 1kHz
        then a sample_rate of 0 will generate an interrupt at 1kHz, while a sample_rate of 9 will generate
        an interrupt 1 out of every 10 reads, or 100Hz. The sensor registers are updated at the same
        time an interrupt is (or would have been, if interrupts are disabled) so in these examples,
        0 would lead to the sensors being updated at 1kHz and 9 would lead to 100Hz.
 \return true if all writes are successful, false otherwise.
*/
bool MPU::configure(uint8_t gyro_scale, uint8_t acc_scale, uint8_t bandwidth, uint8_t sample_rate) {
  //Set the sample rate
  if(!write(SMPLRT_DIV,sample_rate))
  {return false;}
  //Set External sync (none, 0x00) and Low Pass Filter
  if(!write(CONFIG,(0b000<<3) | // ext_sync_set - function disabled
             ((bandwidth & 0b111)<<0))) //gyro bandwidth - lower number is higher bandwidth. Only 3 bits wide so grab those bits from input
  {return false;}
  //Turn off all gyro self-test and set the gyro scale
  if(!write(GYRO_CONFIG,(0 << 7) | //x self test off
             (0 << 6) | //y self test off
             (0 << 5) | //z self test off
             ((gyro_scale & 0b11) << 3))) //Gyro scale, full range is +/-(250deg/s)*2^gyro_scale, so lower is more sensitive
  {return false;}
  //Turn off all acc  self-test and set the acc  scale
  if(!write(ACCEL_CONFIG,(0 << 7) | //x self test off
             (0 << 6) | //y self test off
             (0 << 5) | //z self test off
             ((acc_scale  & 0b11) << 3))) //Gyro scale, full range is +/-(2g)*2^acc_scale, so lower is more sensitive
  {return false;}
  return true;
}

/** Read configuration registers
 \param buf buffer to hold registers. Must be at least 128 bytes, and
    config registers are read into that buffer. Not all registers are read,
    so buffer space for those that are not are left as-is. The result is
    that the index into the buffer directly maps to the address of the register,
    so that register 0x75 (whoami) is found at index 0x75
 \return true if all reads are successful, false otherwise.
*/
bool MPU::readConfig(char buf[]) {
  //First few bytes written out are gyro configuration. Gyro is
  //configured in the constructor for HardwarePiInterfaceArduino,
  //so the configuration is finished by this point.
  //First are registers 0x13 to 0x1F inclusive (13 bytes)
  if(!read(0x13,buf+0x13,13)) return false;
  //Next are registers 0x23 to 0x3A inclusive (24 bytes)
  if(!read(0x23,buf+0x23,24)) return false;
  //Next are registers 0x3B to 0x48 inclusive (14 bytes). These are the sensor values
  if(!read(0x3B,buf+0x3B,14)) return false;
  //Next are registers 0x67 to 0x6C inclusive (6 bytes)
  if(!read(0x67,buf+0x67,6)) return false;
  //Next are registers 0x72 to 0x75 inclusive (4 bytes)
  if(!read(0x72,buf+0x72,4)) return false;
  //Next are registers 0x77 to 0x78 inclusive (2 bytes)
  if(!read(0x77,buf+0x77,2)) return false;
  //Next are registers 0x7A to 0x7B inclusive (2 bytes)
  if(!read(0x7A,buf+0x7A,2)) return false;
  //Finally are registers 0x7D to 0x7E inclusive (2 bytes)
  if(!read(0x7D,buf+0x7D,2)) return false;
  return true;
}

/** \copydoc MPU::readGyro(int16_t&,int16_t&,int16_t&)
 * \internal
 * Implemented by reading 8 bytes the I2C bus, starting at register 0x41
 */
bool MPU::readGyro(int16_t& gx, int16_t& gy, int16_t& gz) {
  char buf[sizeof(int16_t)*3];
  if(!read(GYRO_XOUT_H,buf,sizeof(buf))) return false;
  gx=readBuf_be<int16_t>(buf,0);
  gy=readBuf_be<int16_t>(buf,2);
  gz=readBuf_be<int16_t>(buf,4);
  return true;
}

/** \copydoc MPU::readGyro(int16_t&,int16_t&,int16_t&,int16_t&)
 * \internal
 * Implemented by reading 8 bytes the I2C bus, starting at register 0x41
 */
bool MPU::readGyro(int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t) {
  char buf[sizeof(int16_t)*4];
  if(!read(TEMP_OUT_H,buf,sizeof(buf))) return false;
  t =readBuf_be<int16_t>(buf,0);
  gx=readBuf_be<int16_t>(buf,2);
  gy=readBuf_be<int16_t>(buf,4);
  gz=readBuf_be<int16_t>(buf,6);
  return true;
}

/** \copydoc MPU::readAcc(int16_t&,int16_t&,int16_t&)
 * \internal
 * Implemented by reading 6 bytes the I2C bus, starting at register 0x3B
 */
bool MPU::readAcc(int16_t& ax, int16_t& ay, int16_t& az) {
  char buf[sizeof(int16_t)*3];
  if(!read(ACCEL_XOUT_H,buf,sizeof(buf))) return false;
  ax=readBuf_be<int16_t>(buf,0);
  ay=readBuf_be<int16_t>(buf,2);
  az=readBuf_be<int16_t>(buf,4);
  return true;
}

/** \copydoc MPU::readAcc(int16_t&,int16_t&,int16_t&,int16_t&)
 * \internal
 * Implemented by reading 8 bytes the I2C bus, starting at register 0x3B
 */
bool MPU::readAcc(int16_t& ax, int16_t& ay, int16_t& az, int16_t& t) {
  char buf[sizeof(int16_t)*4];
  if(!read(ACCEL_XOUT_H,buf,sizeof(buf))) return false;
  ax=readBuf_be<int16_t>(buf,0);
  ay=readBuf_be<int16_t>(buf,2);
  az=readBuf_be<int16_t>(buf,4);
  t =readBuf_be<int16_t>(buf,6);
  return true;
}
/** \copydoc MPU::readAcc(int16_t&,int16_t&,int16_t&)
 * \internal
 * Implemented by reading 14 bytes the I2C bus, starting at register 0x3B
 */
bool MPU::readMPU(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t) {
  char buf[sizeof(int16_t)*7];
  if(!read(ACCEL_XOUT_H,buf,sizeof(buf))) return false;
  ax=readBuf_be<int16_t>(buf, 0);
  ay=readBuf_be<int16_t>(buf, 2);
  az=readBuf_be<int16_t>(buf, 4);
  t =readBuf_be<int16_t>(buf, 6);
  gx=readBuf_be<int16_t>(buf, 8);
  gy=readBuf_be<int16_t>(buf,10);
  gz=readBuf_be<int16_t>(buf,12);
  return true;
}

/** Start the AK8963 and initialize the configuration. Do anything necessary to init the part. Bus is available at this point.
*/
bool AK89xx::begin() {
  bool success;
  uint8_t who=whoami(success);
  if(!success||who!=0x48) {
//    printf("Failed: %02x=whoami(%s)",who,success?"true":"false");
    return false;
  }
  configure();
  return true;
}

/** Configure the AK8963
 \param mode Sensor read mode selected from this table:
   mode      | description
  ---------- | ---------------------
       0     | Power-down mode. Majority of part is asleep, but I2C bus is active
       1     | Single measurement mode - Take one measurement then automatically go to mode 0
       2     | Continuous measurement mode 1 - Sample sensor at 8Hz
       6     | Continuous measurement mode 2 - Sample sensor at 100Hz
       4     | External trigger mode - not useful since external trigger not exposed in MPU9xxx
       8     | Self-test mode. Take self-test measurement, then automatically go to mode 0
      15     | Fuse ROM mode - read Fuse ROM but don't do anything else
 \param bit16 If true, sensor read values are 16-bit, else 14-bit.
 \return true if all writes are successful, false otherwise.
*/
bool AK89xx::configure(uint8_t mode, bool bit16) {
  char cntl1=(mode & 0x0F) |
             ((bit16?1:0)<<4);
  //Transition to mode 0 first
  if(!write(CNTL1,0)) return false;
  //Wait at least 100us for transition to happen
  usleep(100);
  //Transition to desired mode
  if(!write(CNTL1,cntl1)) return false;
  return true;
}

/** Read configuration registers
 \param buf buffer to hold registers. Must be at least 19 bytes, and
    config registers are read into that buffer. Not all registers are read,
    so buffer space for those that are not are left as-is. The result is
    that the index into the buffer directly maps to the address of the register,
    so that register 0x00 (whoami) is found at index 0x00
 \return true if all reads are successful, false otherwise.

 Note: This sets the CNTL1 register to make the Fuse ROM registers visible. This will
       disrupt any measurement in progress. If any of the I2C accesses fail, the part
       may be left in Fuse ROM mode, and will not take any further measurements.
*/
bool AK89xx::readConfig(char buf[]) {
  //Read CNTL1 to get the current mode
  char cntl1_old;
  const char cntl1_new=0x0F; //Put the part into Fuse ROM access mode, so that the sensitivity settings are readable
  if(!read(CNTL1,&cntl1_old,1)) return false;
  if(!write(CNTL1,cntl1_new)) return false;
  //First few bytes written out are gyro configuration. Gyro is
  //configured in the constructor for HardwarePiInterfaceArduino,
  //so the configuration is finished by this point.
  //First are registers 0x00 to 0x0C inclusive (13 bytes). This inculdes sensor values.
  if(!read(0x00,buf+0x00,13)) return false;
  //Next are registers 0x0F to 0x12 inclusive (4 bytes).
  if(!read(0x0F,buf+0x0F,4)) return false;
  if(!write(CNTL1,0)) return false; //Set the mode to powerdown
  usleep(100); //Wait for mode change
  if(!write(CNTL1,cntl1_old)) return false; //Set the mode back to the original mode
  return true;
}

/** \copydoc AK::read(int16_t&,int16_t&,int16_t&)
 * \internal
 * Reads the ST1 register, and returns false if data is not ready yet. If
 * the data IS ready, read the sensor values and ST2.
 * @return true if the sensor is read successfully. False if not.
 *         If the data isn't ready yet, return false and zero in all data values
 *         If the sensor saturates, return false, but return the (presumably
 *         nonzero) data that we got.
 */
bool AK89xx::read(int16_t& bx, int16_t& by, int16_t& bz) {
  bx=by=bz=0; //Clear out the return values
  char buf[7]; //Enough data for the three 16-bit sensor values and one status register
  if(!read(ST1,buf,1)) return false;
  if((buf[0] & 0x01) == 0x00) return false; //Data not ready yet
  if(!read(HXL,buf,sizeof(buf))) return false;
  bx=readBuf_le<int16_t>(buf, 0);
  by=readBuf_le<int16_t>(buf, 2);
  bz=readBuf_le<int16_t>(buf, 4);
  return (buf[6] & (1<<3)) == 0; //ST2 is in buf[6]. Bit 3 is magnetic sensor overflow (analog saturate).
}

