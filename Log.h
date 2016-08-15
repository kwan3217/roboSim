#ifndef LOG_H_
#define LOG_H_

/** Logging class. Default implementation ignores logging messages. This is suitable for something with no filesystem
 *  like an Arduino, but an actual implementation will be needed to record data in a file or set of files. Can cover
 *  any quadrant on the logging space, text/binary and single/multiple streams.
 */
class Log {
public:
  virtual void startPacket(int apid) {};
  virtual void write(int8_t value) {};
  virtual void write(int16_t value) {};
  virtual void write(int32_t value) {};
  virtual void write(uint8_t value) {};
  virtual void write(uint16_t value) {};
  virtual void write(uint32_t value) {};
  virtual void write(float value) {};
  virtual void write(char* value, int len) {};
  virtual void write(char* value) {};
  virtual void endPacket(int apid) {};
};

#endif /* LOG_H_ */
