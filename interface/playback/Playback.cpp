/*
 * Playback.cpp
 */
#include "stdio.h"
#include "robot.h"
#include "Playback.h"

double NMEAPlayback::checkPPS(){
  return t;
}
bool NMEAPlayback::checkNavChar(){
  bool result=feof(inf)==0;
  return result;
}
char NMEAPlayback::readChar(){
  char c;
  fread(&c,1,1,inf);
  return c;
}
double NMEAPlayback::time(){
  return t;
}

