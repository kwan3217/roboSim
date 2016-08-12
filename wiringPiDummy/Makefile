libwiringPi.so: wiringPi.o
	ld -G $^ -o $@
	sudo ${RM} /usr/local/lib/libwiringPi.so
	sudo ${RM} /usr/local/include/wiringPi.h
	sudo ln -sv ${PWD}/libwiringPi.so /usr/local/lib
	sudo ln -sv ${PWD}/wiringPi.h /usr/local/include

wiringPi.o: wiringPi.c wiringPi.h
	gcc -c $< -o $@

clean:
	${RM} *.o *.so

