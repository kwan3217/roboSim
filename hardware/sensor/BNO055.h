#ifndef BNO055_h
#define BNO055_h
#include <inttypes.h>
#include "I2C.h"
#include <errno.h>

/** Driver for BNO055 motion processing unit
 *
 */
class BNO055 {
  //Register addresses. In hexadecimal because the majority of the register map
  //document uses hexadecimal registers
  // Register addresses higher than 0x7F are on the second page, so
  // set page to 1 and use address-0x80 as the address
  static const int CHIP_ID          = 0x00;
  static const int ACC_ID           = 0x01;
  static const int MAG_ID           = 0x02;
  static const int GYR_ID           = 0x03;
  static const int SW_REV_LSB       = 0x04;
  static const int SW_REV_MSB       = 0x05;
  static const int BL_REV_ID        = 0x06;
  static const int PAGE_ID0         = 0x07;
  static const int ACC_DATA_X_LSB   = 0x08;
  static const int ACC_DATA_X_MSB   = 0x09;
  static const int ACC_DATA_Y_LSB   = 0x0A;
  static const int ACC_DATA_Y_MSB   = 0x0B;
  static const int ACC_DATA_Z_LSB   = 0x0C;
  static const int ACC_DATA_Z_MSB   = 0x0D;
  static const int MAG_DATA_X_LSB   = 0x0E;
  static const int MAG_DATA_X_MSB   = 0x0F;
  static const int MAG_DATA_Y_LSB   = 0x10;
  static const int MAG_DATA_Y_MSB   = 0x11;
  static const int MAG_DATA_Z_LSB   = 0x12;
  static const int MAG_DATA_Z_MSB   = 0x13;
  static const int GYR_DATA_X_LSB   = 0x14;
  static const int GYR_DATA_X_MSB   = 0x15;
  static const int GYR_DATA_Y_LSB   = 0x16;
  static const int GYR_DATA_Y_MSB   = 0x17;
  static const int GYR_DATA_Z_LSB   = 0x18;
  static const int GYR_DATA_Z_MSB   = 0x19;
  static const int EUL_HEADING_LSB  = 0x1A;
  static const int EUL_HEADING_MSB  = 0x1B;
  static const int EUL_ROLL_LSB     = 0x1C;
  static const int EUL_ROLL_MSB     = 0x1D;
  static const int EUL_PITCH_LSB    = 0x1E;
  static const int EUL_PITCH_MSB    = 0x1F;
  static const int QUA_DATA_W_LSB   = 0x20;
  static const int QUA_DATA_W_MSB   = 0x21;
  static const int QUA_DATA_X_LSB   = 0x22;
  static const int QUA_DATA_X_MSB   = 0x23;
  static const int QUA_DATA_Y_LSB   = 0x24;
  static const int QUA_DATA_Y_MSB   = 0x25;
  static const int QUA_DATA_Z_LSB   = 0x26;
  static const int QUA_DATA_Z_MSB   = 0x27;
  static const int LIA_DATA_X_LSB   = 0x28;
  static const int LIA_DATA_X_MSB   = 0x29;
  static const int LIA_DATA_Y_LSB   = 0x2A;
  static const int LIA_DATA_Y_MSB   = 0x2B;
  static const int LIA_DATA_Z_LSB   = 0x2C;
  static const int LIA_DATA_Z_MSB   = 0x2D;
  static const int GRV_DATA_X_LSB   = 0x2E;
  static const int GRV_DATA_X_MSB   = 0x2F;
  static const int GRV_DATA_Y_LSB   = 0x30;
  static const int GRV_DATA_Y_MSB   = 0x31;
  static const int GRV_DATA_Z_LSB   = 0x32;
  static const int GRV_DATA_Z_MSB   = 0x33;
  static const int TEMP             = 0x34;
  static const int CALIB_STAT       = 0x35;
  static const int ST_RES           = 0x36;
  static const int INT_STA          = 0x37;
  static const int SYS_CLK_STATUS   = 0x38;
  static const int SYS_STATUS       = 0x39;
  static const int SYS_ERR          = 0x3A;
  static const int UNIT_SEL         = 0x3B;
/* reserved 0x3C*/
  static const int OPR_MODE         = 0x3D;
  static const int PWR_MODE         = 0x3E;
  static const int SYS_TRIGGER      = 0x3F;
  static const int TEMP_SOURCE      = 0x40;
  static const int AXIS_MAP_CONFIG  = 0x41;
  static const int AXIS_MAP_SIGN    = 0x42;
/* reserved 0x43-0x54*/
  static const int ACC_OFFSET_X_LSB = 0x55;
  static const int ACC_OFFSET_X_MSB = 0x56;
  static const int ACC_OFFSET_Y_LSB = 0x57;
  static const int ACC_OFFSET_Y_MSB = 0x58;
  static const int ACC_OFFSET_Z_LSB = 0x59;
  static const int ACC_OFFSET_Z_MSB = 0x5A;
  static const int MAG_OFFSET_X_LSB = 0x5B;
  static const int MAG_OFFSET_X_MSB = 0x5C;
  static const int MAG_OFFSET_Y_LSB = 0x5D;
  static const int MAG_OFFSET_Y_MSB = 0x5E;
  static const int MAG_OFFSET_Z_LSB = 0x5F;
  static const int MAG_OFFSET_Z_MSB = 0x60;
  static const int GYR_OFFSET_X_LSB = 0x61;
  static const int GYR_OFFSET_X_MSB = 0x62;
  static const int GYR_OFFSET_Y_LSB = 0x63;
  static const int GYR_OFFSET_Y_MSB = 0x64;
  static const int GYR_OFFSET_Z_LSB = 0x65;
  static const int GYR_OFFSET_Z_MSB = 0x66;
  static const int ACC_RADIUS_LSB   = 0x67;
  static const int ACC_RADIUS_MSB   = 0x68;
  static const int MAG_RADIUS_LSB   = 0x69;
  static const int MAG_RADIUS_MSB   = 0x6A;
/* reserved 0x6B-0x7F*/
/* reserved 0x80-0x86*/
  static const int PAGE_ID1          = 0x87;
  static const int ACC_CONFIG       = 0x88;
  static const int MAG_CONFIG       = 0x89;
  static const int GYR_CONFIG0      = 0x8A;
  static const int GYR_CONFIG1      = 0x8B;
  static const int ACC_SLEEP_CONFIG = 0x8C;
  static const int GYR_SLEEP_CONFIG = 0x8D;
/* reserved 0x8E*/
  static const int INT_MSK          = 0x8F;
  static const int INT_EN           = 0x90;
  static const int ACC_AM_THRES     = 0x91;
  static const int ACC_INT_SETTINGS = 0x92;
  static const int ACC_HG_DURATION  = 0x93;
  static const int ACC_HG_THRES     = 0x94;
  static const int ACC_NM_THRES     = 0x95;
  static const int ACC_NM_SET       = 0x96;
  static const int GYR_INT_SETING   = 0x97;
  static const int GYR_HR_X_SET     = 0x98;
  static const int GYR_DUR_X        = 0x99;
  static const int GYR_HR_Y_SET     = 0x9A;
  static const int GYR_DUR_Y        = 0x9B;
  static const int GYR_HR_Z_SET     = 0x9C;
  static const int GYR_DUR_Z        = 0x9D;
  static const int GYR_AM_THRES     = 0x9E;
  static const int GYR_AM_SET       = 0x9F;
/* reserved 0xA0-0xCF*/
  static const int UNIQUE_ID0       = 0xD0;
  static const int UNIQUE_ID1       = 0xD1;
  static const int UNIQUE_ID2       = 0xD2;
  static const int UNIQUE_ID3       = 0xD3;
  static const int UNIQUE_ID4       = 0xD4;
  static const int UNIQUE_ID5       = 0xD5;
  static const int UNIQUE_ID6       = 0xD6;
  static const int UNIQUE_ID7       = 0xD7;
  static const int UNIQUE_ID8       = 0xD8;
  static const int UNIQUE_ID9       = 0xD9;
  static const int UNIQUE_IDA       = 0xDA;
  static const int UNIQUE_IDB       = 0xDB;
  static const int UNIQUE_IDC       = 0xDC;
  static const int UNIQUE_IDD       = 0xDD;
  static const int UNIQUE_IDE       = 0xDE;
  static const int UNIQUE_IDF       = 0xDF;
/* reserved 0xE0-0xFF*/
  static const int CHIP_ID_DAPAT=0xA0;
public:
  bool readConfig(char buf[]);
private:
  I2C_t bus;
  bool begin();
  virtual bool write(uint8_t addr, uint8_t val) {return writeI2Creg(bus, ADDRESS, addr, val);};
  virtual uint8_t read(uint8_t addr, bool& success) {return readI2Creg(bus, ADDRESS, addr, success);};
  virtual int16_t read16(uint8_t addr, bool& success) {return readI2Creg_be<int16_t>(bus, ADDRESS, addr, success);};
  virtual bool read(uint8_t addr, char buf[], int len) {return readI2Creg(bus,ADDRESS,addr,buf,len);};
  virtual uint8_t getPage(bool& success) {return read(7,success);};
  virtual bool setPage(uint8_t page) {return write(7,page);};
  bool readConfig(char buf[], char first, char last);
public:
  static const int ADDRESS=0x28;///< 7-bit I2C address of BNO055
  bool begin(I2C_t Lbus) {
    bus=Lbus;
    return begin();
  }
//  int errno=-1; //If an error occurs that causes a function to return
                //false, this will contain the line number of the check
                //that caused the problem
};

#endif
