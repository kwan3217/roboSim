#ifndef LOG_H_
#define LOG_H_

#include <inttypes.h>

/** Logging class. Default implementation ignores logging messages. This is suitable for something with no filesystem
 *  like an Arduino, but an actual implementation will be needed to record data in a file or set of files. Can cover
 *  any quadrant on the logging space, text/binary and single/multiple streams.
 *
 *  A packet consists of a packet identifier followed by a fixed structure of data. All packets with the same identifier
 *  should have the same structure. We follow the CCSDS specification and call this packet identifer the APID, Application
 *  Process ID
 *
 *  Text/binary is pretty much what it sounds like. Text could be something like CSV or NMEA sentences. Binary is something
 *  like CCSDS packets, where the binary data is just concatenated and stuffed in the packet stream.
 *
 *  For single/multiple streams, use the APID to identify which stream this packet goes into. So for instance, startPacket
 *  would look up which stream this packet goes into and makes subsequent writes go to it.
 */
class Log {
private:
  void getRecordPath();
protected:
  static char recordPath[256];
public:
  Log() {getRecordPath();};
  virtual ~Log() {};
  /** Start a packet with a particular APID
   \param[in] apid APID to use for this packet
   */
  virtual void startPacket(int apid) {};
  /** Add a signed 8-bit value to the packet
   \param[in] value value to write
   */
  virtual void write(int8_t value) {};
  /** Add a signed 16-bit value to the packet
   \param[in] value value to write
   */
  virtual void write(int16_t value) {};
  /** Add a signed 32-bit value to the packet
   \param[in] value value to write
   */
  virtual void write(int32_t value) {};
  /** Add an unsigned 8-bit value to the packet
   \param[in] value value to write
   */
  virtual void write(uint8_t value) {};
  /** Add an unsigned 16-bit value to the packet
   \param[in] value value to write
   */
  virtual void write(uint16_t value) {};
  /** Add an unsigned 32-bit value to the packet
   \param[in] value value to write
   */
  virtual void write(uint32_t value) {};
  /** Add a single-precision floating-point value to the packet
   \param[in] value value to write
   */
  virtual void write(float value) {};
  /** Add a byte buffer to the packet
   \param[in] value value to write
   \param[in] len number of bytes to write
   */
  virtual void write(char* value, int len) {};
  /** Add a null-terminated string to the packet
   \param[in] value value to write
   */
  virtual void write(char* value) {};
  /** Start a packet with a particular APID
   \param[in] apid APID that was used for this packet, used to verify we are finishing the same packet we started.
   */
  virtual void endPacket(int apid) {};
  virtual void startDescribe(int apid) {};
  virtual void endDescribe(int apid) {};
  virtual void describe(char* name, int type) {};
  //These follow IDL types just as a reference
  static const int t_u8 = 1; ///< Field type is unsigned 8-bit int;
  static const int t_i16= 2; ///< Field type is signed 16-bit int;
  static const int t_i32= 3; ///< Field type is signed 32-bit int;
  static const int t_u16=12; ///< Field type is unsigned 16-bit int;
  static const int t_u32=13; ///< Field type is unsigend 32-bit int;
  static const int t_float=4; ///< Field type is IEEE754 single-precision 32-bit float
  static const int t_double=5;///< Field type is IEEE754 double-precision 64-bit float
};

#endif /* LOG_H_ */
