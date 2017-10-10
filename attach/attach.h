#ifndef attach_h
#define attach_h

#include "Log.h"

extern char _binary_attach_tbz_start[];
extern char _binary_attach_tbz_end[];
extern char _binary____attach_parseYukari_py_start[];
extern char _binary____attach_parseYukari_py_end[];

void dumpAttach(Log& pkt, int pktSize=64);
void dumpParse(Log& pkt);

#endif
