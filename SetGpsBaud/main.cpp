#include <stdio.h>

int main() {
  FILE* ouf=fopen("/dev/ttyAMA0","w");
  if(!ouf) {
    printf("failed");
  }
  fprintf(ouf,"$PMTK251,115200*1F\x0D\x0A");
  fclose(ouf);
}

