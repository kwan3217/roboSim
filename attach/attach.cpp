#include "attach.h"

void dumpAttach(Log& pkt, int pktSize) {
  int dumpSize=_binary_attach_tbz_end-_binary_attach_tbz_start;
  for(int i=0;i<dumpSize;i+=pktSize) {
    pkt.start(Log::Apids::dump,"Dump");
    pkt.write(_binary_attach_tbz_start+i,i+pktSize>dumpSize?dumpSize-i:pktSize,"DumpData");
    pkt.end();
  }
}

void dumpParse(Log& pkt) {
  int dumpSize=_binary____attach_parseYukari_py_end-_binary____attach_parseYukari_py_start;
  pkt.start(Log::Apids::parse,"parseYukari");
  pkt.write(_binary____attach_parseYukari_py_start,dumpSize,"python3code");
  pkt.end();
}
