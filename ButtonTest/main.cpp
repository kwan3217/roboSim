#include "HardwarePi.h"
//#include "Simulator.h"
//#include "OpenLoopGuidance.h"
#include <iostream>

HardwarePiInterfaceArduino interface;

void setup() {

}

void loop() {
  printf("%d\n",interface.button());
}

int main() {
  setup();
  for(;;) {
    loop();
  }
}
