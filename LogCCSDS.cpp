#include "LogCCSDS.h"

LogCCSDS::LogCCSDS(const char* basename, int LdocApid, int LmetaDocApid):docApid(LdocApid),metaDocApid(LmetaDocApid) {
  char filename[256];
  snprintf(filename,sizeof(filename)-1,"%s/%s",recordPath,basename);
  stream=fopen(filename,"w");
  setbuf(stream,nullptr); //Turn off buffering for this file
}

LogCCSDS::~LogCCSDS() {
  fclose(stream);
}

void LogCCSDS::writeDoc(int type, const char* fieldName) {
  if(fieldName==nullptr) return;
  if(hasDoc[pktApid]) return;
  start(docBuf,docPtr,docApid);
  write(docBuf,docPtr,(uint16_t)pktApid);
  write(docBuf,docPtr,(uint16_t)pktPtr);
  write(docBuf,docPtr,(uint8_t)type);
  write(docBuf,docPtr,fieldName);
  end(docBuf,docPtr,docApid);
}

void LogCCSDS::start(char* buf, int& ptr, int apid) {
  writeBuf_be<uint16_t>(buf,0,apid);
  writeBuf_be<uint16_t>(buf,2,0xC000 | seq[apid]);
  seq[apid]++;
  if(seq[apid]>=0xC000) seq[apid]=0;
  ptr=6;
}

void LogCCSDS::end(char* buf, int& ptr, int apid) {
  hasDoc[apid]=true;
  writeBuf_be<uint16_t>(buf,4,ptr-7);
  fwrite(buf,ptr,1,stream);
}



void LogCCSDS::metaDoc() {
  metaDoc("This file contains CCSDS packets as described in CCSDS 133.0-B-1 "
		   "with updates.");
  metaDoc("https://public.ccsds.org/Pubs/133x0b1c2.pdf");
  metaDoc("Each packet starts with a six-byte header which holds the packet "
		   "type (APID), sequence number and length.");
  metaDoc("All numbers in a CCSDS packet are stored Big-Endian.");
  metaDoc("Packets start with a 16-bit unsigned integer, upper 5 bytes are "
		   "version and type, lower 11 bytes are the APID");
  metaDoc("Next 16-bit number is 0xC000 (flags indicating unfragmented packet) "
		   "followed by a sequence number which increments for each APID and "
		   "wraps in 14 bits.");
  metaDoc("Next 16-bit number is length of packet minus 7 since every packet "
		   "has a 6-byte header and must have at least 1 byte of payload.");
  metaDoc("Packets of apid %d contain this English meta-documentation. ",metaDocApid);
  metaDoc("Packets of apid %d describe the detailed format of the other "
		  "packets.",docApid);
  metaDoc("In those packets, the first field is a 16-bit number in the payload is the "
		  "packet APID being described.");
  metaDoc("The second field is a 16-bit number in the payload is the position in the "
		  "packet of the field being described.");
  metaDoc("This value is zero if the whole packet is being named.");
  metaDoc("Otherwise the position is a zero-based count of bytes from the beginning "
		  "of the packet, and will never be less than six.");
  metaDoc("This value will be unreliable for fields written after a variable length"
		  "string or binary, but usually such fields are the last field in a packet.");
  metaDoc("For packets with multiple variable-length fields, the position written "
		  "follows from the first packet of this APID to be written");
  metaDoc("The third field is an 8-bit number in the payload is the type of the field"
		  "from the following table: ");
  metaDoc("0x%02x: This name describes the whole packet, not any one field",0);
  metaDoc("0x%02x: t_u8 (8-bit unsigned integer)",t_u8);
  metaDoc("0x%02x: t_i16 (16-bit signed integer)",t_i16);
  metaDoc("0x%02x: t_i32 (32-bit signed integer)",t_i32);
  metaDoc("0x%02x: t_u16 (16-bit unsigned integer)",t_u16);
  metaDoc("0x%02x: t_u32 (32-bit unsigned integer)",t_u32);
  metaDoc("0x%02x: t_float (32-bit IEEE-754 floating point)",t_float);
  metaDoc("0x%02x: t_double (64-bit IEEE-754 floating point)",t_double);
  metaDoc("0x%02x: t_string (ASCII text)",t_string);
  metaDoc("0x%02x: t_binary (unformatted data dump)",t_binary);
  metaDoc("The fourth field is an ASCII text string with the name of the field");
  metaDoc("For all strings and binary data, no length information is included.");
  metaDoc("If the string or binary is the only such field in the packet, its"
		  "length can be deduced from the packet length.");
  metaDoc("Otherwise the packet will need some indication of the length.");
  metaDoc("ASCII text might have a null-termination, but binary data will usually need"
		  "a prefix length, which is described in its own field.");
  metaDoc("Although the Space Packet Protocol allows it, this packet writer does "
		  "not leave gaps between fields -- IE all fields are contiguous");
  metaDoc("Although the Space Packet Protocol allows fields on non-byte boundaries, "
		  "this packet writer always writes whole-byte fields.");
}
