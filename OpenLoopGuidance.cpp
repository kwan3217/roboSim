#include "OpenLoopGuidance.h"
#include <cstdio>

void OpenLoopGuidance::control() {
  if(t[currentRow]<interface.time()) {
	printf("Controller: t=%f, i=%d, t_cmd=%f, s=%c, s=%d\n",interface.time(),currentRow,t[currentRow],servoChannel[currentRow],servoSetting[currentRow]);
	if(servoChannel[currentRow]=='S') {
	  interface.steering.write(servoSetting[currentRow]);
	} else {
	  interface.throttle.write(servoSetting[currentRow]);
	}
	currentRow++;
  }
}


