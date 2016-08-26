#Set the names of the tools we will be using, to make it easy to change, for instance
#if we want to do cross-compiling on a host, or want to switch between gcc and clang.
CC = gcc
CXX = g++
OBJDUMP=objdump
OBJCOPY=objcopy
ATTACH = *.cpp *.h
ATTACH+= wiringPiDummy/*.c wiringPiDummy/*.h wiringPiDummy/Makefile
ATTACH+=Yukari4.fzz
ATTACH+=Doxyfile Makefile
ATTACH+=MPU9250registers.ods
ATTACH+=Robodometer.ino

#Detect system type so that we can build attach.o for the host system 
#(no cross-compile here).
SYSTYPE:=$(shell uname -m)
ifeq ($(SYSTYPE),armv6l)
BFDO := elf32-littlearm
BFDB := arm
else
BFDO := elf64-x86-64
BFDB := i386
endif


### Set the compiler options, used during the call to g++ to make each .o file. ###
#Turn on link-time optimization. In the compile phase, this writes extra information in
#the .o files (effectively the full parse tree) so that the linker has this information.
#In the link phase, the linker is then able to do its own optimization, especially 
#inlining of functions across .o files, in the final executable.
#CXXFLAGS= -flto

#Turn on debugging information
CXXFLAGS+=-g

#Set the language standard to C++14, the most current released standard at time of writing
CXXFLAGS+=-std=c++14

#Set the optimization level. Recommend one of these levels
#-O0 - no optimization at all. This tends to write really stupid code, such as writing
#      a register to memory then immediately reading it back, explicitly casting all 
#      boolean expressions to 0 or 1 then checking for equality with 1, etc.
#-Og - Optimize the debugging experience. Do the optimizations which don't change the 
#      flow of code too much, so that the generated code matches structure of the source
#      code and it can be followed in a debugger.
#-O3 - Turn on almost all the optimizations.
CXXFLAGS+=-O0

#Turn on dependency generation. This makes the preprocessor make a list of which source 
#and header files include, and therefore depend on, which headers. This is done recursively,
#so that any depth of include file tree can be handled. It does this in a form that make
#can read.
#Specifically:
# -MMD says only do user includes (#include "...") not system includes (#include <...>)
# -MP  says make a phony rule for each file even if it doesn't have any dependencies. 
#      In the case where a header is deleted because a human decides it is no longer needed,
#      this avoids having make complain about missing files and refusing to run to update 
#      the dependencies.
# -MF  says write the dependency file for foo.o file in .dep/foo.o.d
CXXFLAGS+=-MMD -MP -MF .dep/$(@F).d

# List of all the executable images that can be made
EXE = RoboSim.exe RoboPi.exe buttonTest.exe recordOdometer.exe i2c_echo.exe recordGyro.exe testCompassNeedle.exe PiCompassNeedle.exe

# List of all extended listings that can be made
LSS = $(EXE:%.exe=%.lss)

# We make the extended listings, which have their executables as dependencies, to get all the executables
all: $(EXE)

# Don't include this in all, since doxygen is not present on the Pi.
html: Doxyfile
	doxygen

#Remove all intermediate and target files
clean:
	$(RM) -r $(EXE) $(LSS) *.e *.s *.o html latex attach.tbz

#Rule for making extended listing. We use objdump to dump several sections out. Some are tables, some
#are code, which is disassembled, some are data which is dumped in hex. If there is an archive attached
#to the executable, we extract it from the file, then list its contents.
%.lss: %.exe
	$(OBJDUMP) -h $< > $@
	$(OBJDUMP) -S -j .text $< | c++filt | tail -n +4 >> $@
	$(OBJDUMP) -s -j .vtable -j .rodata_str -j .rodata -j .ctors $< |c++filt |  tail -n +4 >> $@
	$(OBJCOPY) -O binary -j .source $< $<.tmp.tbz
	if [ -s $<.tmp.tbz ] ;then  echo "Source tarball contents:" >> $@ ; tar tvf $<.tmp.tbz >> $@; fi
	$(RM) -f $<.tmp.tbz 
	$(OBJDUMP) -s                        -j .data $< |c++filt |  tail -n +4 >> $@
	$(OBJDUMP) -S -j .text_lib $< |c++filt | tail -n +4 >> $@
	$(OBJDUMP) -s -j .ARM.exidx -j .ARM.extab -j .glue -j .vtable_lib -j .rtti_info -j .rtti_name -j .rodata_str_lib -j .rodata_lib -j .ctors_lib $< | tail -n +4 >> $@
	$(OBJDUMP) -s                        -j .data_lib $< |c++filt |  tail -n +4 >> $@
	$(OBJDUMP) -t $< | grep ^[0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f] |  sort | c++filt >> $@

attach.o: attach.tbz
	echo Systype: $(SYSTYPE)
	echo BFDO: $(BFDO)
	echo BFDB: $(BFDB)
	$(OBJCOPY) -I binary -O $(BFDO) $< $@ --rename-section .data=.source -B $(BFDB)

attach.tbz: $(ATTACH)
	tar jcvhf $@ $(sort $^)

#Rule to link an executable. Whenever another rule specifies a .exe with dependency .o files but no
#recipe, this recipe is used.
%.exe:
	$(CXX) -g -o $@ $^ -L /usr/local/lig -lwiringPi

#Rule to compile an object file. Whenever another rule specifies a .o file as a dependency, and there
#is a matching .cpp file, this recipe is used to make it. This rule runs the compiler twice to 
#generate three files:
#  foo.e - preprocessor output
#  foo.s - annotated assembly output
#  foo.o - binary object output
#The .s and .o files are generated in the same pass. I haven't figured out how to get the compiler
#to do the .e file in the same pass yet.
#The first pass uses -E and the pattern substitution formula to make a .e file for each .o file to be
#made.
#The second pass uses -c to compile but not link, generating a .o file. -Wa passes arguments to the
#assembler pass to write annotated assembly to a .s file for each .o file. 
%.o: %.cpp
	#$(CXX) $(CPPFLAGS) $(CXXFLAGS) -E $< -o $(@:.o=.e)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Wa,-a,-ad,-aln=$(@:.o=.s) -c $< -o $@
	#$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

#Main executable rules. Dependencies are the .o files needed. Those .o files themselves depend on
#the source code, so we just need to list the .o files and make will figure out which source
#files and how to make the .o files. Is called .exe by convention, even though it isn't necessary
#in Unix. Keep the .exe extension to make it easy to identify executables.
RoboSim.exe: MainSimRobo.o Simulator.o roboBrain.o

RoboPi.exe: RoboPiMain.o HardwarePi.o OpenLoopGuidance.o Simulator.o MPU.o

buttonTest.exe: buttonTest.o HardwarePi.o MPU.o

recordOdometer.exe: recordOdometer.o HardwarePi.o MPU.o Simulator.o OpenLoopGuidance.o

recordGyro.exe: recordGyro.o HardwarePi.o MPU.o Log.o LogCCSDS.o LogCSV.o LogRawBinary.o dump.o attach.o

testCompassNeedle.exe: testCompassNeedle.o Simulator.o compassNeedle.o

PiCompassNeedle.exe: PiCompassNeedle.o HardwarePi.o compassNeedle.o MPU.o

i2c_echo.exe: i2c_echo.o

testCompassNeedle.csv: testCompassNeedle.exe
	./$< > $@

#Include the dependency stuff. This makes the .dep folder if necessary. Included last so that 
#these are never the default rules.
-include $(shell mkdir .dep 2>/dev/null) $(ALLDEP)

