#include "attach.h"

void dumpAttach(Log& pkt, int apid, int pktSize) {
  int dumpSize=_binary_attach_tbz_end-_binary_attach_tbz_start;
  for(int i=0;i<dumpSize;i+=pktSize) {
    pkt.start(apid,"Dump");
    pkt.write(_binary_attach_tbz_start+i,i+pktSize>dumpSize?dumpSize-i:pktSize,"DumpData");
    pkt.end();
  }
}
