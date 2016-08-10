all: RoboSim.exe RoboPi.exe

MainSimRobo.o: MainSimRobo.cpp robot.h Simulator.h roboBrain.h
	g++ -g -c -std=c++14 -o $@ $<

Simulator.o: Simulator.cpp robot.h Simulator.h
	g++ -g -c -std=c++14 -o $@ $<

roboBrain.o: roboBrain.cpp roboBrain.h robot.h Simulator.h
	g++ -g -c -std=c++14 -o $@ $<

#Main executable rule. Dependencies are each .o file, one for each .cpp file, use wildcard rule above to make them.
#Is called .exe by convention, even though it isn't necessary in Unix. Keep the .exe extension to make it easy to identify executables.
RoboSim.exe: MainSimRobo.o Simulator.o roboBrain.o
	g++ -g -o $@ $^ # Link in debug mode to an executable, output name from $@, input is all named .o files ($^)

HardwarePi.o: HardwarePi.cpp HardwarePi.h robot.h
	g++ -g -c -std=c++14 -o $@ $<

RoboPiMain.o: RoboPiMain.cpp HardwarePi.h robot.h
	g++ -g -c -std=c++14 -o $@ $<

RoboPiSetupLoop.o: RoboPiSetupLoop.cpp HardwarePi.h robot.h
	g++ -g -c -std=c++14 -o $@ $<

RoboPi.exe: RoboPiMain.o RoboPiSetupLoop.o HardwarePi.o roboBrain.o
	g++ -g -o $@ $^

html: Doxyfile
	doxygen

#Remove all compiled files
clean:
	$(RM) -r RoboSim.exe RoboPi.exe *.o html latex
