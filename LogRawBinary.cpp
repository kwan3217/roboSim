#include "LogRawBinary.h"

LogRawBinary::LogRawBinary(const char* basename) {
  char filename[256];
  snprintf(filename,sizeof(filename)-1,"%s/%s",recordPath,basename);
  stream=fopen(filename,"w");
  setbuf(stream,nullptr); //Turn off buffering for this file
}

LogRawBinary::~LogRawBinary() {
  fclose(stream);
}

