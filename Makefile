%.o: %.cpp
	g++ -g -c -o $@ $<

RoboSim.exe: MainSimRobo.o
	g++ -g -o $@ $<
