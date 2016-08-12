#include "HardwarePi.h"

void MPU::begin(uint8_t gyro_scale, uint8_t acc_scale, uint8_t bandwidth, uint8_t sample_rate) {
  write(0x6B,(0 << 7) | (0 << 6) | (0 << 5) | (0x01 << 0));
  usleep(50000);
  //Do anything necessary to init the part. Bus is available at this point.
  //Set External sync (none, 0x00) and Low Pass Filter
  bandwidth&=0x07;
  uint8_t lpf_setup=(0x00<<3) | (bandwidth<<0);
  write(0x1A,lpf_setup);
  //Turn off all gyro self-test and set the gyro scale
  uint8_t gyro_config=(0 << 7) | (0 << 6) | (0 << 5) | ((gyro_scale & 0x03) << 3);
  write(0x1B,gyro_config);
  //Turn off all acc  self-test and set the acc  scale
  uint8_t acc_config =(0 << 7) | (0 << 6) | (0 << 5) | ((acc_scale  & 0x03) << 3);
  write(0x1C,acc_config);
  //Set the sample rate
  write(0x19,sample_rate);
  //Set up I2C passthrough, so that the magnetic sensor on a 9150 can be accessed. Also set int levels
  uint8_t int1_config=(0 << 7) | //0 - INT line is active high
                      (0 << 6) | //0 - INT line is push-pull (no pullup needed, can't attach another int line and do a wired and)
                      (0 << 5) | //0 - LATCH_INT_EN is disabled, int line emits a 50us pulse. This is detectable by the capture line but doesn't need any action to reset
                      (1 << 4) | //1 - INT_RD_CLEAR on any read. If you want int status, read that register first on receiving an int
                      (0 << 3) | //0 - Frame Sync input - designed to trigger a read in sync with an external event, like a video frame capture. Not used in any Kwan design yet
                      (0 << 2) | //0 - Frame sync interrupt enabled - when set, a pulse on the FSYNC input to the 60x0 will cause an interrupt on the INT line to the host
                      (1 << 1) ; //1 - I2C_BYPASS_EN - bypass enable. When set to 1, the auxillary bus (including the mag sensor on the 9150) is just tied to the main
                                 //                    bus so that the host can see the auxillary sensor(s) directly.
  write(0x37,int1_config);
  //In order for passthrough to work, the 9150 I2C bus master must be disabled, but this is default, so we don't do anything with it
}

bool MPUI2C::readGyro(int16_t& gx, int16_t& gy, int16_t& gz) {

}
bool MPUI2C::readAcc(int16_t& ax, int16_t& ay, int16_t& az) {

}
bool MPUI2C::read(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz, int16_t& t) {

}

void MPUI2C::write(uint8_t addr, uint8_t val) {

}

uint8_t MPUI2C::read(uint8_t addr) {

}

int16_t MPUI2C::read16(uint8_t addr) {

}
