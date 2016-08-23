#ifndef buffer_h
#define buffer_h
/** @file */

/** Extract a little-endian integer from a byte buffer
 * \tparam T type of integer to extract
 * \param buf byte buffer
 * \param ofs offset of first byte in number (will go into least significant byte)
 */
template<typename T>
inline T readBuf_le(char buf[], int ofs) {
  T result=0;
  for(int i=0;i<sizeof(T);i++) result |= (T(buf[ofs+i]) & 0xFF) << i*8;
  return result;
}

/** Extract a big-endian integer from a byte buffer
 * \tparam T type of integer to extract
 * \param buf byte buffer
 * \param ofs offset of first byte in number (will go into most significant byte)
 */
template<typename T>
inline T readBuf_be(char buf[], int ofs) {
  T result=0;
  for(int i=0;i<sizeof(T);i++) result |= (T(buf[ofs+i]) & 0xFF) << (sizeof(T)-1-i)*8;
  return result;
}

/** Insert a little-endian integer into a byte buffer
 * \tparam T type of integer to insert
 * \param buf byte buffer
 * \param ofs offset of first byte in number (least significant byte will go here)
 */
template<typename T>
inline void writeBuf_le(char buf[], int ofs, T data) {
  for(int i=0;i<sizeof(T);i++) buf[ofs+i] = char((data >> i*8) & 0xFF);
}

/** Insert a big-endian integer into a byte buffer
 * \tparam T type of integer to insert
 * \param buf byte buffer
 * \param ofs offset of first byte in number (most significant byte will go here)
 */
template<typename T>
inline void writeBuf_be(char buf[], int ofs, T data) {
  for(int i=0;i<sizeof(T);i++) buf[ofs+i] = char((data >> (sizeof(T)-1-i)*8) & 0xFF);
}

#endif
