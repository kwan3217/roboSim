#include <wiringPi.h>
#include <stdio.h>
int main(){
	wiringPiSetupGpio();
	pinMode(22, OUTPUT);
		while(true){
		int pin18 = digitalRead(17);
		digitalWrite(22, pin18);
		printf("%c", pin18==0?'_':'X');
		}
	}
