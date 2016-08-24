#ifndef I2C_h
#define I2C_h

/** @file
 *
 * Routines to read and write I2C with the "register" protocol.
 *
 * In this protocol, the slave
 * is modeled as a series of 8-bit registers, each with an 8-bit address, for a total of 256 in the
 * space. The slave also maintains an address pointer. Using a feature called "burst mode", each time
 * a register is accessed, the address pointer is incremented to point to the next register. This means
 * that each consecutive register doesn't have to be addressed. The exact details depend on the slave.
 *
 * To send data from the master to a register in the slave, the master addresses the slave for
 * write, then writes the register address, then zero or more bytes of data. Sending zero bytes after
 * the register address sets the address pointer but takes no further action. To receive data from the
 * slave, first address the slave register as above, then address the slave for read and read one or
 * more bytes. If the master knows that the slave register address pointer is already correct, it can
 * just do the read.
 *
 * The routines in this file encapsulate this protocol and make it easy to read or write single bytes
 * or burst mode.
 * */

#include <linux/i2c-dev.h> //For I2C_SLAVE constant
#include <sys/ioctl.h>     //For ioctl()
#include <unistd.h>        //for read() and write()
#include "buffer.h"

typedef int I2C_t;

/** Write a buffer to the I2C bus
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to write to
 * \param buf buffer to write
 * \param buf number of bytes to write
 * \return true if write succeeded, false otherwise
 */
inline bool writeI2C(I2C_t bus,  uint8_t slaveaddr, char* buf, int len) {
  ioctl(bus,I2C_SLAVE,slaveaddr);
  bool result=(len==write(bus,buf,len));
  return result;
}

/** Write a byte to a specified register in a slave on an I2C bus
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to write to
 * \param regaddr register address to write to
 * \param data value to write
 * \return true if write succeeded, false otherwise
 */
inline bool writeI2Creg(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr, uint8_t data) {
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
 * \return true if write succeeded, false otherwise
 */
template<typename T>
inline int writeI2Creg_le(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr, T data) {
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
 * \return true if write succeeded, false otherwise
 */
template<typename T>
inline bool writeI2Creg_be(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr, T data) {
  char buf[sizeof(T)+1];
  buf[0]=regaddr;
  writeBuf_be<T>(buf,1,data);
  bool result=writeI2C(bus,slaveaddr,buf,sizeof(T)+1);
  return result;
}

/** Read a buffer from the I2C bus
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to write to
 * \param buf buffer to read to
 * \param buf number of bytes to read
 * \return true if write succeeded, false otherwise
 */
inline bool readI2C(I2C_t bus,  uint8_t slaveaddr, char* buf, int len) {
  ioctl(bus,I2C_SLAVE,slaveaddr);
  bool result=(len==read(bus,buf,len));
  return result;
}

/** Read a byte from a specified register in a slave on an I2C bus
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to read from
 * \param regaddr register address to read from
 * \param success true if write succeeded, false otherwise
 * \return data value that was read
 */
inline uint8_t readI2Creg(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr, bool& success) {
  char buf=regaddr;
  if(!writeI2C(bus,slaveaddr,&buf,1)) {
//    printf("Addressing device failed, bus=%d slaveaddr=%02x\n",bus,slaveaddr);
    success=false;
    return 0;
  }
  if(read(bus,&buf,1)!=1) {
//    printf("Reading device failed\n");
    success=false;
    return 0;
  }
  success=true;
  return buf;
}

/** Read a series of consecutive byte registers in a slave on an I2C bus
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to read from
 * \param regaddr register address of first register to read from
 * \param buf buffer to read into
 * \param len number of bytes to read
 * \return true if read succeeded, false if not
 */
inline bool readI2Creg(I2C_t bus, uint8_t slaveaddr, uint8_t regaddr, char* buf, int len) {
  buf[0]=regaddr;
  if(!writeI2C(bus,slaveaddr,buf,1)) {
//    printf("Addressing device failed\n");
    return false;
  }
  if(!readI2C(bus,slaveaddr,buf,len)) {
//    printf("Reading device failed\n");
    return false;
  }
  return true;
}

/** Read a value from a multi-byte little-endian register in a slave on an I2C bus
 * \tparam T integer type of register
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to write to
 * \param regaddr register address of first (least significant) byte to write to
 * \param success true if write succeeded, false otherwise
 * \return value in that multi-byte register
 */
template<typename T>
inline T readI2Creg_le(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr, bool& success) {
  char buf[sizeof(T)];
  readI2Creg(bus,slaveaddr,regaddr,buf,sizeof(T));
  return readBuf_le<T>(buf,0);
}

/** Read a value from a multi-byte big-endian register in a slave on an I2C bus
 * \tparam T integer type of register
 * \param bus stream representing I2C bus
 * \param slaveaddr slave address to write to
 * \param regaddr register address of first (most significant) byte to write to
 * \param success true if write succeeded, false otherwise
 * \return value in that multi-byte register
 */
template<typename T>
inline T readI2Creg_be(I2C_t bus,  uint8_t slaveaddr, uint8_t regaddr, bool& success) {
  char buf[sizeof(T)];
  readI2Creg(bus,slaveaddr,regaddr,buf,sizeof(T));
  return readBuf_be<T>(buf,0);
}

#endif
