#include <wiringPi.h>

int wiringPiFailure (int fatal, const char *message, ...) {}

// Core wiringPi functions

struct wiringPiNodeStruct *wiringPiFindNode (int pin) {}
struct wiringPiNodeStruct *wiringPiNewNode  (int pinBase, int numPins) {}

int  wiringPiSetup       (void) {}
int  wiringPiSetupSys    (void) {}
int  wiringPiSetupGpio   (void) {}
int  wiringPiSetupPhys   (void) {}

void pinModeAlt          (int pin, int mode) {}
void pinMode             (int pin, int mode) {}
void pullUpDnControl     (int pin, int pud) {}
int  digitalRead         (int pin) {}
void digitalWrite        (int pin, int value) {}
void pwmWrite            (int pin, int value) {}
int  analogRead          (int pin) {}
void analogWrite         (int pin, int value) {}

// PiFace specifics 
//	(Deprecated)

int  wiringPiSetupPiFace (void) {}
int  wiringPiSetupPiFaceForGpioProg (void) {}	// Don't use this - for gpio program only

// On-Board Raspberry Pi hardware specific stuff

         int  piBoardRev          (void) {}
         void piBoardId           (int *model, int *rev, int *mem, int *maker, int *overVolted) {}
         int  wpiPinToGpio        (int wpiPin) {}
         int  physPinToGpio       (int physPin) {}
         void setPadDrive         (int group, int value) {}
         int  getAlt              (int pin) {}
         void pwmToneWrite        (int pin, int freq) {}
         void digitalWriteByte    (int value) {}
unsigned int  digitalReadByte     (void) {}
         void pwmSetMode          (int mode) {}
         void pwmSetRange         (unsigned int range) {}
         void pwmSetClock         (int divisor) {}
         void gpioClockSet        (int pin, int freq) {}

// Interrupts
//	(Also Pi hardware specific)

int  waitForInterrupt    (int pin, int mS) {}
int  wiringPiISR         (int pin, int mode, void (*function)(void)) {}

// Threads

int  piThreadCreate      (void *(*fn)(void *)) {}
void piLock              (int key) {}
void piUnlock            (int key) {}

// Schedulling priority

int piHiPri (const int pri) {}

// Extras from arduino land

void         delay             (unsigned int howLong) {}
void         delayMicroseconds (unsigned int howLong) {}
unsigned int millis            (void) {}
unsigned int micros            (void) {}

