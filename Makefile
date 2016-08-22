CC = gcc
CXX = g++
CXXFLAGS= -g -std=c++14
OBJDUMP=objdump
OBJCOPY=objcopy

EXES = RoboSim.exe RoboPi.exe buttonTest.exe recordOdometer.exe i2c_echo.exe recordGyro.exe testCompassNeedle.exe PiCompassNeedle.exe

all: $(EXES)

html: Doxyfile
	doxygen

#Remove all compiled files
clean:
	$(RM) -r $(EXES) *.o html latex

#Rule for making extended listing
%.lss: %.exe
	$(OBJDUMP) -h $< > $@
	$(OBJDUMP) -S -j .vectors $< |tail -n +4 >> $@
	$(OBJDUMP) -S -j .text $< | c++filt | tail -n +4 >> $@
	$(OBJDUMP) -s -j .vtable -j .rodata_str -j .rodata -j .ctors $< |c++filt |  tail -n +4 >> $@
	$(OBJCOPY) -O binary -j .source $< $<.tmp.zpaq
	if [ -s $<.tmp.zpaq ] ;then  echo "Source tarball contents:" >> $@ ; zpaq110 xn $<.tmp.zpaq $<.tmp.cpio ; cpio -ivt < $<.tmp.cpio >> $@ ; $(RM) $<.tmp.cpio; fi
	$(RM) -f $<.tmp.zpaq $<.tmp.cpio
	$(OBJDUMP) -s                        -j .data $< |c++filt |  tail -n +4 >> $@
	$(OBJDUMP) -S -j .text_lib $< |c++filt | tail -n +4 >> $@
	$(OBJDUMP) -s -j .ARM.exidx -j .ARM.extab -j .glue -j .vtable_lib -j .rtti_info -j .rtti_name -j .rodata_str_lib -j .rodata_lib -j .ctors_lib $< | tail -n +4 >> $@
	$(OBJDUMP) -s                        -j .data_lib $< |c++filt |  tail -n +4 >> $@
	$(OBJDUMP) -t $< | grep ^[0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f] | c++filt | sort >> $@

#We make a lot of use of the default recipe:
#	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<
#We specify the dependencies, but Make figures out to use this default recipe
MainSimRobo.o: MainSimRobo.cpp robot.h Simulator.h roboBrain.h

compassNeedle.o: compassNeedle.cpp robot.h compassNeedle.h

testCompassNeedle.o: testCompassNeedle.cpp robot.h Simulator.h compassNeedle.h

PiCompassNeedle.o: testCompassNeedle.cpp robot.h Simulator.h compassNeedle.h

Simulator.o: Simulator.cpp robot.h Simulator.h

roboBrain.o: roboBrain.cpp roboBrain.h robot.h Simulator.h

#Main executable rule. Dependencies are each .o file, one for each .cpp file, use wildcard rule above to make them.
#Is called .exe by convention, even though it isn't necessary in Unix. Keep the .exe extension to make it easy to identify executables.
RoboSim.exe: MainSimRobo.o Simulator.o roboBrain.o
	$(CXX) -g -o $@ $^ # Link in debug mode to an executable, output name from $@, input is all named .o files ($^)

HardwarePi.o: HardwarePi.cpp HardwarePi.h robot.h

MPU.o: MPU.cpp HardwarePi.h robot.h

RoboPiMain.o: RoboPiMain.cpp HardwarePi.h robot.h

OpenLoopGuidance.o: OpenLoopGuidance.cpp OpenLoopGuidance.h HardwarePi.h robot.h

RoboPi.exe: RoboPiMain.o HardwarePi.o OpenLoopGuidance.o Simulator.o MPU.o
	$(CXX) -g -o $@ $^ -L /usr/local/lib -lwiringPi

buttonTest.o: buttonTest.cpp HardwarePi.h robot.h

recordOdometer.o: recordOdometer.cpp HardwarePi.h robot.h

recordGyro.o: recordGyro.cpp HardwarePi.h robot.h

buttonTest.exe: buttonTest.o HardwarePi.o MPU.o
	$(CXX) -g -o $@ $^ -L /usr/local/lib -lwiringPi

recordOdometer.exe: recordOdometer.o HardwarePi.o MPU.o Simulator.o OpenLoopGuidance.o
	$(CXX) -g -o $@ $^ -L /usr/local/lib -lwiringPi

recordGyro.exe: recordGyro.o HardwarePi.o MPU.o Simulator.o OpenLoopGuidance.o
	$(CXX) -g -o $@ $^ -L /usr/local/lib -lwiringPi

i2c_echo.exe: i2c_echo.c
	${CC} -g -o $@ $^ -std=c99

testCompassNeedle.exe: testCompassNeedle.o Simulator.o compassNeedle.o
	$(CXX) -g -o $@ $^ # Link in debug mode to an executable, output name from $@, input is all named .o files ($^)

PiCompassNeedle.exe: PiCompassNeedle.o HardwarePi.o compassNeedle.o MPU.o
	$(CXX) -g -o $@ $^ -L /usr/local/lib -lwiringPi # Link in debug mode to an executable, output name from $@, input is all named .o files ($^)

testCompassNeedle.csv: testCompassNeedle.exe
	./$< > $@


