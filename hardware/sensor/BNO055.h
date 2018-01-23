#ifndef BNO055_h
#define BNO055_h
#include <inttypes.h>
#include "I2C.h"
#include <errno.h>
#include "float.h"

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
  static const int SIC_MATRIX_0_LSB = 0x43;
  static const int SIC_MATRIX_0_MSB = 0x44;
  static const int SIC_MATRIX_1_LSB = 0x45;
  static const int SIC_MATRIX_1_MSB = 0x46;
  static const int SIC_MATRIX_2_LSB = 0x47;
  static const int SIC_MATRIX_2_MSB = 0x48;
  static const int SIC_MATRIX_3_LSB = 0x49;
  static const int SIC_MATRIX_3_MSB = 0x4A;
  static const int SIC_MATRIX_4_LSB = 0x4B;
  static const int SIC_MATRIX_4_MSB = 0x4C;
  static const int SIC_MATRIX_5_LSB = 0x4D;
  static const int SIC_MATRIX_5_MSB = 0x4E;
  static const int SIC_MATRIX_6_LSB = 0x4F;
  static const int SIC_MATRIX_6_MSB = 0x50;
  static const int SIC_MATRIX_7_LSB = 0x51;
  static const int SIC_MATRIX_7_MSB = 0x52;
  static const int SIC_MATRIX_8_LSB = 0x53;
  static const int SIC_MATRIX_8_MSB = 0x54;
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
/* reserved 0x6B*/
/* reserved 0x6C*/
/* reserved 0x6D*/
/* reserved 0x6E*/
/* reserved 0x6F*/
/* reserved 0x70*/
/* reserved 0x71*/
/* reserved 0x72*/
/* reserved 0x73*/
/* reserved 0x74*/
/* reserved 0x75*/
/* reserved 0x76*/
/* reserved 0x77*/
/* reserved 0x78*/
/* reserved 0x79*/
/* reserved 0x7A*/
/* reserved 0x7B*/
/* reserved 0x7C*/
/* reserved 0x7D*/
/* reserved 0x7E*/
/* reserved 0x7F*/
/* reserved 0x80*/
/* reserved 0x81*/
/* reserved 0x82*/
/* reserved 0x83*/
/* reserved 0x84*/
/* reserved 0x85*/
/* reserved 0x86*/
  static const int PAGE_ID1         = 0x07;
  static const int ACC_CONFIG       = 0x08;
  static const int MAG_CONFIG       = 0x09;
  static const int GYR_CONFIG0      = 0x0A;
  static const int GYR_CONFIG1      = 0x0B;
  static const int ACC_SLEEP_CONFIG = 0x0C;
  static const int GYR_SLEEP_CONFIG = 0x0D;
/* reserved 0x8E*/
  static const int INT_MSK          = 0x0F;
  static const int INT_EN           = 0x10;
  static const int ACC_AM_THRES     = 0x11;
  static const int ACC_INT_SETTINGS = 0x12;
  static const int ACC_HG_DURATION  = 0x13;
  static const int ACC_HG_THRES     = 0x14;
  static const int ACC_NM_THRES     = 0x15;
  static const int ACC_NM_SET       = 0x16;
  static const int GYR_INT_SETING   = 0x17;
  static const int GYR_HR_X_SET     = 0x18;
  static const int GYR_DUR_X        = 0x19;
  static const int GYR_HR_Y_SET     = 0x1A;
  static const int GYR_DUR_Y        = 0x1B;
  static const int GYR_HR_Z_SET     = 0x1C;
  static const int GYR_DUR_Z        = 0x1D;
  static const int GYR_AM_THRES     = 0x1E;
  static const int GYR_AM_SET       = 0x1F;
/* reserved 0xA0*/
/* reserved 0xA1*/
/* reserved 0xA2*/
/* reserved 0xA3*/
/* reserved 0xA4*/
/* reserved 0xA5*/
/* reserved 0xA6*/
/* reserved 0xA7*/
/* reserved 0xA8*/
/* reserved 0xA9*/
/* reserved 0xAA*/
/* reserved 0xAB*/
/* reserved 0xAC*/
/* reserved 0xAD*/
/* reserved 0xAE*/
/* reserved 0xAF*/
/* reserved 0xB0*/
/* reserved 0xB1*/
/* reserved 0xB2*/
/* reserved 0xB3*/
/* reserved 0xB4*/
/* reserved 0xB5*/
/* reserved 0xB6*/
/* reserved 0xB7*/
/* reserved 0xB8*/
/* reserved 0xB9*/
/* reserved 0xBA*/
/* reserved 0xBB*/
/* reserved 0xBC*/
/* reserved 0xBD*/
/* reserved 0xBE*/
/* reserved 0xBF*/
/* reserved 0xC0*/
/* reserved 0xC1*/
/* reserved 0xC2*/
/* reserved 0xC3*/
/* reserved 0xC4*/
/* reserved 0xC5*/
/* reserved 0xC6*/
/* reserved 0xC7*/
/* reserved 0xC8*/
/* reserved 0xC9*/
/* reserved 0xCA*/
/* reserved 0xCB*/
/* reserved 0xCC*/
/* reserved 0xCD*/
/* reserved 0xCE*/
/* reserved 0xCF*/
  static const int UNIQUE_ID0       = 0x50;
  static const int UNIQUE_ID1       = 0x51;
  static const int UNIQUE_ID2       = 0x52;
  static const int UNIQUE_ID3       = 0x53;
  static const int UNIQUE_ID4       = 0x54;
  static const int UNIQUE_ID5       = 0x55;
  static const int UNIQUE_ID6       = 0x56;
  static const int UNIQUE_ID7       = 0x57;
  static const int UNIQUE_ID8       = 0x58;
  static const int UNIQUE_ID9       = 0x59;
  static const int UNIQUE_IDA       = 0x5A;
  static const int UNIQUE_IDB       = 0x5B;
  static const int UNIQUE_IDC       = 0x5C;
  static const int UNIQUE_IDD       = 0x5D;
  static const int UNIQUE_IDE       = 0x5E;
  static const int UNIQUE_IDF       = 0x5F;
/* reserved 0xE0*/
/* reserved 0xE1*/
/* reserved 0xE2*/
/* reserved 0xE3*/
/* reserved 0xE4*/
/* reserved 0xE5*/
/* reserved 0xE6*/
/* reserved 0xE7*/
/* reserved 0xE8*/
/* reserved 0xE9*/
/* reserved 0xEA*/
/* reserved 0xEB*/
/* reserved 0xEC*/
/* reserved 0xED*/
/* reserved 0xEE*/
/* reserved 0xEF*/
/* reserved 0xF0*/
/* reserved 0xF1*/
/* reserved 0xF2*/
/* reserved 0xF3*/
/* reserved 0xF4*/
/* reserved 0xF5*/
/* reserved 0xF6*/
/* reserved 0xF7*/
/* reserved 0xF8*/
/* reserved 0xF9*/
/* reserved 0xFA*/
/* reserved 0xFB*/
/* reserved 0xFC*/
/* reserved 0xFD*/
/* reserved 0xFE*/
/* reserved 0xFF*/

  static const int CHIP_ID_DAPAT=0xA0;
private:
  I2C_t bus;
  bool begin();
  bool write(uint8_t addr, uint8_t val) {return writeI2Creg(bus, ADDRESS, addr, val);};
  uint8_t read(uint8_t addr, bool& success) {return readI2Creg(bus, ADDRESS, addr, success);};
  int16_t read16(uint8_t addr, bool& success) {return readI2Creg_le<int16_t>(bus, ADDRESS, addr, success);};
  bool read(uint8_t addr, char buf[], int len) {return readI2Creg(bus,ADDRESS,addr,buf,len);};
  uint8_t getPage(bool& success) {return read(7,success);};
  bool setPage(uint8_t page) {return write(7,page);};
  bool readConfig(char buf[], char first, char last);
  void readVecRaw(int addr, int n, int16_t val[]) {
	for(int i=0;i<n;i++) {
	  val[i]=readBuf_le<int16_t>(sampleBuf,addr-SAMPLE_BUF_BEGIN+i*2);
	}
  };
  void readVec(int addr, fp scale, int n, fp val[]) {
	for(int i=0;i<n;i++) {
	  val[i]=fp(readBuf_le<int16_t>(sampleBuf,addr-SAMPLE_BUF_BEGIN+i*2))/scale;
	}
  };
  static const int SAMPLE_BUF_BEGIN=PAGE_ID0;
  static const int SAMPLE_BUF_END  =TEMP;
  char sampleBuf[SAMPLE_BUF_END-SAMPLE_BUF_BEGIN+1];
public:
  char configBuf[256];
  static const int ADDRESS=0x28;///< 7-bit I2C address of BNO055
  bool begin(I2C_t Lbus) {bus=Lbus;return begin();}
  bool readConfig();
  bool sample() {return read(SAMPLE_BUF_BEGIN,sampleBuf,sizeof(sampleBuf));};
  /** Return total acceleration in body frame in m/s^2 */
  void readAcc     (fp      val[]) {readVec   (ACC_DATA_X_LSB ,100.0,3,val);};
  void readAccRaw  (int16_t val[]) {readVecRaw(ACC_DATA_X_LSB ,      3,val);};
  /** Return magnetic field in uT */
  void readMag     (fp      val[]) {readVec   (MAG_DATA_X_LSB , 16.0,3,val);};
  void readMagRaw  (int16_t val[]) {readVecRaw(MAG_DATA_X_LSB ,      3,val);};
  /** Return rotation rate in body frame in rad/s */
  void readGyro    (fp      val[]) {readVec   (GYR_DATA_X_LSB ,900.0,3,val);};
  void readGyroRaw (int16_t val[]) {readVecRaw(GYR_DATA_X_LSB ,      3,val);};
  /** Return Euler angles heading, pitch, roll in radians */
  void readEuler   (fp      val[]) {readVec   (EUL_HEADING_LSB,900.0,3,val);};
  void readEulerRaw(int16_t val[]) {readVecRaw(EUL_HEADING_LSB,      3,val);};
  /** Return quaternion */
  void readQuat    (fp      val[]) {readVec   (QUA_DATA_W_LSB ,1<<14,4,val);};
  void readQuatRaw (int16_t val[]) {readVecRaw(QUA_DATA_W_LSB ,      4,val);};
  /** Return non-gravitational acceleration */
  void readLia     (fp      val[]) {readVec   (LIA_DATA_X_LSB ,100.0,3,val);};
  void readLiaRaw  (int16_t val[]) {readVecRaw(LIA_DATA_X_LSB ,      3,val);};
  /** Return gravitational acceleration */
  void readGrv     (fp      val[]) {readVec   (GRV_DATA_X_LSB ,100.0,3,val);};
  void readGrvRaw  (int16_t val[]) {readVecRaw(GRV_DATA_X_LSB ,      3,val);};
  /** Return temperature */
  int8_t readTempRaw() {return int8_t(sampleBuf[TEMP-SAMPLE_BUF_BEGIN]);};
  fp readTemp() {return fp(readTempRaw());};
  /** Return all values (say for stuffing in a packet) */
  void readAllRaw(char buf[]) {for(int i=0;i<sizeof(sampleBuf);i++) buf[i]=sampleBuf[i];};
};

#endif
