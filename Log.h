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
   \param[in] ptkName Name of this packet, used for self-documentation purposes
   \param[in] apid APID to use for this packet
   */
  virtual void start(char* pktName, int apid)=0;
  /** Add a signed 8-bit value to the packet
   \param[in] fieldname Name of this field, used for self-documentation purposes
   \param[in] value value to write
   */
  virtual void write(char* fieldName, int8_t value)=0;
  /** Add a signed 16-bit value to the packet
   \param[in] fieldname Name of this field, used for self-documentation purposes
   \param[in] value value to write
   */
  virtual void write(char* fieldName, int16_t value)=0;
  /** Add a signed 32-bit value to the packet
   \param[in] fieldname Name of this field, used for self-documentation purposes
   \param[in] value value to write
   */
  virtual void write(char* fieldName, int32_t value)=0;
  /** Add an unsigned 8-bit value to the packet
   \param[in] fieldname Name of this field, used for self-documentation purposes
   \param[in] value value to write
   */
  virtual void write(char* fieldName, uint8_t value)=0;
  /** Add an unsigned 16-bit value to the packet
   \param[in] fieldname Name of this field, used for self-documentation purposes
   \param[in] value value to write
   */
  virtual void write(char* fieldName, uint16_t value)=0;
  /** Add an unsigned 32-bit value to the packet
   \param[in] fieldname Name of this field, used for self-documentation purposes
   \param[in] value value to write
   */
  virtual void write(char* fieldName, uint32_t value)=0;
  /** Add a single-precision floating-point value to the packet
   \param[in] fieldname Name of this field, used for self-documentation purposes
   \param[in] value value to write
   */
  virtual void write(char* fieldName, float value)=0;
  /** Add a double-precision floating-point value to the packet
   \param[in] fieldname Name of this field, used for self-documentation purposes
   \param[in] value value to write
   */
  virtual void write(char* fieldName, double value)=0;
  /** Add a byte buffer to the packet. This is intended to be used for arbitrary
      binary data. The length information is not required to be written to the 
      packet, so in packets that are not self-delimiting (say CCSDS) you may
      need to write a length. ASCII packet generators will generally transform
      this to printable characters (such as hex or base85).
   \param[in] fieldname Name of this field, used for self-documentation purposes
   \param[in] value value to write
   \param[in] len number of bytes to write
   */
  virtual void write(char* fieldName, char* value, int len)=0;
  /** Add a null-terminated string to the packet. This is intended to be used
      for ASCII text strings. This will generally be dumped to the packet as-is,
      so be careful if you are using a delimited packet such as CSV or NMEA. The 
      null terminator is not required to be written to the packet, so you may need
      to write a length before or null terminator after this string.
   \param[in] fieldname Name of this field, used for self-documentation purposes
   \param[in] value value to write
   */
  virtual void write(char* fieldName, char* value)=0;
  /** End a packet
   */
  virtual void end()=0;
  //These follow IDL types just as a reference
  static const int t_u8    = 1; ///< Field type is unsigned 8-bit int
  static const int t_i16   = 2; ///< Field type is signed 16-bit int
  static const int t_i32   = 3; ///< Field type is signed 32-bit int
  static const int t_u16   =12; ///< Field type is unsigned 16-bit int
  static const int t_u32   =13; ///< Field type is unsigend 32-bit int
  static const int t_float = 4; ///< Field type is IEEE754 single-precision 32-bit float
  static const int t_double= 5; ///< Field type is IEEE754 double-precision 64-bit float
  static const int t_string= 7; ///< Field is a byte string of arbitrary length. No length info provided, it must be provided elsewhere. Intent is ASCII text
  static const int t_binary= 6; ///< Field is a byte string of arbitrary length. No length info provided, it must be provided elsewhere
};

#endif /* LOG_H_ */
