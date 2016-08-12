#include "HardwarePi.h"

void MPU::begin(uint8_t gyro_scale, uint8_t acc_scale, uint8_t bandwidth, uint8_t sample_rate) {
  write(0x6B, (0 << 7) | // 0 - don't reset
		      (0 << 6) | // 0 - don't sleep
			  (0 << 5) | // 0 - don't cycle and sleep
			  (0b001 << 0));  //clockselect 1 - auto-select best available source (used to be use X gyro)
  usleep(50000); //Wait for this to go out
  //Do anything necessary to init the part. Bus is available at this point.
  //Set External sync (none, 0x00) and Low Pass Filter
  write(0x1A,(0b000<<3) | // ext_sync_set - function disabled
          ((bandwidth & 0b111)<<0)); //gyro bandwidth - lower number is higher bandwidth. Only 3 bits wide so grab those bits from input
  //Turn off all gyro self-test and set the gyro scale
  write(0x1B,(0 << 7) | //x self test off
		     (0 << 6) | //y self test off
			 (0 << 5) | //z self test off
			 ((gyro_scale & 0b11) << 3)); //Gyro scale, full range is +/-(250deg/s)*2^gyro_scale, so lower is more sensitive
  //Turn off all acc  self-test and set the acc  scale
  write(0x1C,(0 << 7) | //x self test off
		     (0 << 6) | //y self test off
			 (0 << 5) | //z self test off
			 ((acc_scale  & 0b11) << 3)); //Gyro scale, full range is +/-(2g)*2^acc_scale, so lower is more sensitive
  //Set the sample rate
  write(0x19,sample_rate);
  //Set up I2C passthrough, so that the magnetic sensor on a 9150 can be accessed. Also set int levels
  write(0x37, (0 << 7) | //0 - INT line is active high
              (0 << 6) | //0 - INT line is push-pull (no pullup needed, can't attach another int line and do a wired and)
              (0 << 5) | //0 - LATCH_INT_EN is disabled, int line emits a 50us pulse. This is detectable by the capture line but doesn't need any action to reset
              (1 << 4) | //1 - INT_RD_CLEAR on any read. If you want int status, read that register first on receiving an int
              (0 << 3) | //0 - Frame Sync input - designed to trigger a read in sync with an external event, like a video frame capture. Not used in any Kwan design yet
              (0 << 2) | //0 - Frame sync interrupt enabled - when set, a pulse on the FSYNC input to the 60x0 will cause an interrupt on the INT line to the host
              (1 << 1)); //1 - I2C_BYPASS_EN - bypass enable. When set to 1, the auxillary bus (including the mag sensor on the 9150) is just tied to the main
                         //                    bus so that the host can see the auxillary sensor(s) directly.);
  //In order for passthrough to work, the 9150 I2C bus master must be disabled, but this is default, so we don't do anything with it
}
/** \copydoc MPU::readGyro(int16_t&,int16_t&,int16_t&)
 * \internal
 * Implemented by reading 6 bytes the I2C bus, starting at register 0x43
 */
bool MPUI2C::readGyro(int16_t& gx, int16_t& gy, int16_t& gz) {
  char buf[sizeof(int16_t)*3];
  readI2Creg(bus,ADDRESS,0x43,buf,sizeof(buf));
  gx=readBuf_be<int16_t>(buf,0);
  gy=readBuf_be<int16_t>(buf,2);
  gz=readBuf_be<int16_t>(buf,4);
  return true;
}
/** \copydoc MPU::readAcc(int16_t&,int16_t&,int16_t&)
 * \internal
 * Implemented by reading 6 bytes the I2C bus, starting at register 0x3B
 */
bool MPUI2C::readAcc(int16_t& ax, int16_t& ay, int16_t& az) {
  char buf[sizeof(int16_t)*3];
  readI2Creg(bus,ADDRESS,0x3B,buf,sizeof(buf));
  ax=readBuf_be<int16_t>(buf,0);
  ay=readBuf_be<int16_t>(buf,2);
  az=readBuf_be<int16_t>(buf,4);
  return true;
}
/** \copydoc MPU::readAcc(int16_t&,int16_t&,int16_t&)
 * \internal
 * Implemented by reading 14 bytes the I2C bus, starting at register 0x3B
 */
bool MPUI2C::read(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t) {
  char buf[sizeof(int16_t)*7];
  readI2Creg(bus,ADDRESS,0x3B,buf,sizeof(buf));
  ax=readBuf_be<int16_t>(buf, 0);
  ay=readBuf_be<int16_t>(buf, 2);
  az=readBuf_be<int16_t>(buf, 4);
  t =readBuf_be<int16_t>(buf, 6);
  gx=readBuf_be<int16_t>(buf, 8);
  gy=readBuf_be<int16_t>(buf,10);
  gz=readBuf_be<int16_t>(buf,12);
  return true;
}

