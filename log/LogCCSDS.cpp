#include "LogCCSDS.h"
#include <stdio.h> //for file reading in attachParse
#include "attach.h"

void LogCCSDS::writeDoc(int type, const char* fieldName) {
  if(fieldName==nullptr) return;
  if(pktApid<Apids::nApid && hasDoc[pktApid]) return;
  start(docBuf,docPtr,Apids::doc);
  write(docBuf,docPtr,(uint16_t)pktApid);
  write(docBuf,docPtr,(uint16_t)pktPtr);
  write(docBuf,docPtr,(uint8_t)type);
  write(docBuf,docPtr,fieldName);
  end(docBuf,docPtr,Apids::doc);
}

void LogCCSDS::start(char* buf, int& ptr, Apids apid) {
  writeBuf_be<uint16_t>(buf,0,apid);
  uint16_t seqword;
  if(apid<Apids::nApid) {
    seqword=0xC000 | seq[apid];
    seq[apid]++;
    if(seq[apid]>=0xC000) seq[apid]=0;
  } else {
    seqword=0x2020;
  }
  writeBuf_be<uint16_t>(buf,2,seqword);
  ptr=6;
}

void LogCCSDS::end(char* buf, int& ptr, Apids apid) {
  if(apid<Apids::nApid) hasDoc[apid]=true;
  if(ptr<=7) {
	printf("Problem with packet: Apid %03x, no data written\n",apid);
//	exit();
  }
  writeBuf_be<uint16_t>(buf,4,ptr-7);
  fwrite(buf,ptr,1,stream);
}

void LogCCSDS::metaDoc() {
  metaDoc("This file contains CCSDS packets as described in CCSDS 133.0-B-1 "
		   "with updates.");
  metaDoc("https://public.ccsds.org/Pubs/133x0b1c2.pdf");
  metaDoc("There are no padding bytes or sync markers between packets.");
  metaDoc("All packets are an integer number of whole bytes.");
  metaDoc("Each packet starts with a six-byte header which holds the packet "
		   "type (APID), sequence number and length.");
  metaDoc("All numbers in a CCSDS packet are stored Big-Endian.");
  metaDoc("Packets start with a 16-bit unsigned integer, upper 5 bits are "
		   "version and type, lower 11 bits are the APID");
  metaDoc("Next is a 16-bit number with the upper 2 bits set (flags indicating unfragmented packet) "
		   "and the lower 14 bits are a sequence number which increments for each APID and "
		   "wraps in 14 bits.");
  metaDoc("Next 16-bit number is length of packet minus 7 since every packet "
		   "has a 6-byte header and must have at least 1 byte of payload.");
  metaDoc("Packets of apid %d encapsulates a python 3 script which can decode this stream into "
                   "separate CSV streams for each apid. The apid and length are correct for this "
                   "type of packet, but other flags may not be.",Apids::parse);
  metaDoc("Packets of apid %d contain this English meta-documentation. ",Apids::metaDoc);
  metaDoc("Packets of apid %d describe the detailed format of the other packets.",Apids::doc);
  metaDoc("In those packets, the first field in the payload is a 16-bit number, the "
		  "packet APID being described.");
  metaDoc("The second field in the payload is a 16-bit number, the position in the "
		  "packet of the field being described.");
  metaDoc("This value is zero if the whole packet is being named.");
  metaDoc("Otherwise the position is a zero-based count of bytes from the beginning "
		  "of the packet, and will never be less than six.");
  metaDoc("This value will be unreliable for fields written after a variable length"
		  "string or binary, but usually such fields are the last field in a packet.");
  metaDoc("For packets with multiple variable-length fields, the position written "
		  "follows from the first packet of this APID to be written");
  metaDoc("The third field in the payload is an 8-bit number, the type of the field"
		  "from the following table: ");
  metaDoc("0x%02x: This name describes the whole packet, not any one field",0);
  metaDoc("0x%02x: t_u8 (8-bit unsigned integer)",t_u8);
  metaDoc("0x%02x: t_i16 (16-bit signed integer)",t_i16);
  metaDoc("0x%02x: t_i32 (32-bit signed integer)",t_i32);
  metaDoc("0x%02x: t_u16 (16-bit unsigned integer)",t_u16);
  metaDoc("0x%02x: t_u32 (32-bit unsigned integer)",t_u32);
  metaDoc("0x%02x: t_float (32-bit IEEE-754 floating point)",t_float);
  metaDoc("0x%02x: t_double (64-bit IEEE-754 floating point)",t_double);
  metaDoc("0x%02x: t_string (UTF-8 text)",t_string);
  metaDoc("0x%02x: t_binary (unformatted data dump)",t_binary);
  metaDoc("The fourth field is a UTF-8 text string with the name of the field.");
  metaDoc("For all strings and binary data, no length information is included.");
  metaDoc("If the string or binary is the only such field in the packet, its "
		  "length can be deduced from the packet length.");
  metaDoc("Otherwise the packet will need some indication of the length.");
  metaDoc("UTF-8 text might have a null-termination, but binary data will usually need"
		  "a prefix length, which is described in its own field.");
  metaDoc("UTF-8 length descriptions, if present, will be of the number of bytes, not code points.");
  metaDoc("Normally text will be ASCII, meaning that the high bit will be cleared.");
  metaDoc("UTF-8 is chosen merely to give a concrete interpretation to set upper-bits");
  metaDoc("Although the Space Packet Protocol allows it, this packet writer does "
		  "not leave gaps between fields -- IE all fields are contiguous");
  metaDoc("Although the Space Packet Protocol allows fields on non-byte boundaries, "
		  "this packet writer always writes whole-byte fields.");
  metaDoc("This file may contain \"dump\" packets. The payload of all these packets "
          "concatenated together in order form a TAR stream compressed with the BZIP2 "
          "algorithm.");
  metaDoc("This stream contains the source code for the program used to write this "
          "packet stream, along with all other files the authors felt necessary to "
          "make a \"self-documenting robot\"");
  dumpParse(*this);
}
