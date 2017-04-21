#include <stdio.h>

int main() {
  FILE* ouf=fopen("/dev/ttyAMA0","rw");
  fprintf(ouf,"$PMTK414*33\x0D\x0A");
  fclose(ouf);
}

