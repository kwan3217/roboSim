#include "BNO055.h"

#include "buffer.h" //for readbuf_be
#include "I2C.h"    //for I2C register access

bool BNO055::begin() {
  //Make sure we are on page 0. Register 0 is reserved on page 1, might
  //be CHIP_ID, but might not. This is a step the Adafruit library
  //doesn't do.
  if(!setPage(0)) {errno=__LINE__; return false;}
  bool readworked;
  //Check if the chip ID is good
  uint8_t id=read(CHIP_ID,readworked); if(!readworked) {errno=__LINE__; return false;}
  if(id!=CHIP_ID_DAPAT) {
    //Hmm, not right ID. Let's wait a bit for it to boot and try again
    usleep(1000000); //Wait a full second for boot
    id=read(CHIP_ID,readworked); if(!readworked) {errno=__LINE__; return false;}
    if(id!=CHIP_ID_DAPAT) {
      //Still not the right chip :(
      {errno=__LINE__; return false;}
    }
  }
  
  //Switch to config mode
  if(!write(OPR_MODE,0x00)) {errno=__LINE__; return false;}
  
  //Adafruit code doesn't do this
  usleep(20000); //Spec says 19ms to go from operational to CONFIG
  
  //Reset
  if(!write(SYS_TRIGGER,0x20)) {errno=__LINE__; return false;}
  while(read(CHIP_ID,readworked)!=CHIP_ID_DAPAT) {
    //if(!readworked) {errno=__LINE__; return false;}
    usleep(10000); //delay 10ms
  }
  usleep(50000); //delay 50ms
  
  //Set to normal power mode
  if(!write(PWR_MODE,0)) {errno=__LINE__; return false;}
  usleep(10000); //delay 10ms
    
  //Commented out code to set units
  
  //commented out code to set axis mapping
  
  //Clear all sys_trigger bits
  if(!write(SYS_TRIGGER,0)) {errno=__LINE__; return false;}
  usleep(10000); //delay 10ms
  
  //Put the processor into NDOF mode. This should tell the fusion
  //processor to do whatever it wants to the sensors.
  if(!write(OPR_MODE,0x0C)) {errno=__LINE__; return false;}
  usleep(20000); //Spec says it takes 7ms to transition from config to
                 //NDOF mode. Adafruit code gives it 20ms
  return true;
}

bool BNO055::readConfig(char buf[],char first, char last) {
  return read(first,buf+first,last-first+1);
}

/** Read configuration registers
 \param buf buffer to hold registers. Must be at least 256 bytes, and
    config registers are read into that buffer. Not all registers are read,
    so buffer space for those that are not are left as-is. The result is
    that the index into the buffer directly maps to the address of the register,
    so that register 0x75 (whoami) is found at index 0x75
 \return true if all reads are successful, false otherwise.
*/
bool BNO055::readConfig(char buf[]) {
  bool result;
  char page=getPage(result);
  if(!result) {errno=__LINE__; return false;}
  if(!setPage(0)) {errno=__LINE__; return false;}
  if(!readConfig(buf,0x00,0x07)) {errno=__LINE__; return false;}
  if(!readConfig(buf,0x08,0x0F)) {errno=__LINE__; return false;}
  if(!readConfig(buf,0x10,0x17)) {errno=__LINE__; return false;}
  if(!readConfig(buf,0x18,0x1F)) {errno=__LINE__; return false;}
  if(!readConfig(buf,0x20,0x27)) {errno=__LINE__; return false;}
  if(!readConfig(buf,0x28,0x2F)) {errno=__LINE__; return false;}
  if(!readConfig(buf,0x30,0x3B)) {errno=__LINE__; return false;}
  if(!readConfig(buf,0x3D,0x42)) {errno=__LINE__; return false;}
  if(!readConfig(buf,0x55,0x6A)) {errno=__LINE__; return false;}
  if(!setPage(1)) {errno=__LINE__; return false;}
  if(!readConfig(buf+0x80,0x07,0x0D)) {errno=__LINE__; return false;}
  if(!readConfig(buf+0x80,0x0F,0x1F)) {errno=__LINE__; return false;}
  if(!readConfig(buf+0x80,0x50,0x5F)) {errno=__LINE__; return false;}
  if(page!=1) {
	  if(!setPage(page)) {errno=__LINE__; return false;}
  }
  return true;
}
