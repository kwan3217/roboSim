#wildcard rule - whenever we need to (re)build a .o file, use this rule to make it from the matching .cpp file
%.o: %.cpp
	g++ -g -c -o $@ $< #Compile in debug mode to an object file (not an executable), output name from rule matching ($@ is the thing to left of : in rule) input from rule matching ($< is first dependency to right of :)

#Main executable rule. Dependencies are each .o file, one for each .cpp file, use wildcard rule above to make them.
#Is called .exe by convention, even though it isn't necessary in Unix. Keep the .exe extension to make it easy to identify executables.
RoboSim.exe: MainSimRobo.o Simulator.o roboBrain.o
	g++ -g -o $@ $^ # Link in debug mode to an executable, output name from $@, input is all named .o files ($^) 
