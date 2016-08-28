#ifndef dump_h
#define dump_h

#include "Log.h"

extern char _binary_attach_tbz_start[];
extern char _binary_attach_tbz_end[];

void dumpAttach(Log& pkt, int apid, int pktSize=64);

#endif
