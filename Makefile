%.o: %.cpp
	g++ -g -c -o $@ $<

RoboSim.exe: MainSimRobo.o Simulator.o
	g++ -g -o $@ $^
