# List C++ source files here.
# use file-extension cpp for C++-files (use extension .cpp)
# Every Loginator sketch uses its own main.cpp, so we reference it here.
#CPPSRC += ../libraries/tarball/tarball.cpp
LIBMAKE+=../libraries/tarball/Makefile
EXTRAINCDIRS+=../libraries/tarball/

# $(ALLSRC) and $(HDEP) are figured in the system Makefile
ATTACH+= $(ALLSRC) $(HDEP) $(LIBMAKE) 
SATTACH=$(sort $(ATTACH))
DDATTACH=$(addprefix $(TARGETBASE)/,$(SATTACH))
TARBALL = attach.tbz
ATTACHOBJ = $(TARBALL).o

$(TARBALL): $(SATTACH)
	tar cjhvf $@ -C .. $(DDATTACH)

ifndef BFDO
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
endif

$(ATTACHOBJ): $(TARBALL)
	$(OBJCOPY) -I binary -O $(BFDO) $< $@ --rename-section .data=.source -B $(BFDB)


