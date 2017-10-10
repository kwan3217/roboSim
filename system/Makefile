#This file is included last, so it takes advantage of any variables defined before this point. Such variables include:

#GENCPP - Generated C++ source files, for instance made by ragel from a .rl 
#         file. These files are not included in the tarball. They should 
#         have the extension .CPP

TARGETPATH:=$(CURDIR)
TARGETBASE=$(notdir $(TARGETPATH))
ifndef TARGET
TARGET=$(TARGETBASE)
endif

ATTACH += ../system/Doxyfile.template
LIBMAKE += Makefile ../system/Makefile
CPPSRC += main.cpp
reduce = $(if $(strip $2),$(call reduce,$1,$(wordlist 2,$(words $2),$2),$(call $1,$(firstword $2),$3)),$3)

#reduce takes two arguments: a function that will be called by reduce and a list to process.  
#The first argument is called with two arguments: each element of the list in reduce's second 
#argument and the result of the previous call to the function.

#To see this in action we can create a list unique function that removes duplicates without reordering:

check_uniq = $(if $(filter $1,$2),$2,$2 $1)

uniq = $(call reduce,check_uniq,$1)
HDEP = $(shell cat .dep/* | sed -n 's/\([^:]*.h\)\s*:/\1/gp' | sort | uniq)

CPPSRC:=$(call uniq,$(CPPSRC))
LIBMAKE:=$(call uniq,$(LIBMAKE))
ALLSRC = $(CPPSRC)
BACKUP = $(addsuffix ~,$(ALLTAR))

# Optimization level, can be [0, 1, 2, 3, s].
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
ifndef OPTLEVEL
OPTLEVEL = 3
endif
OPT = -O$(OPTLEVEL) -fomit-frame-pointer -ffast-math -fno-math-errno

# Debugging format.
# Needed in order to get source code listings in .lst and .lss files
ifndef USE_CLANG
DEBUG = -g
endif
#DEBUG = -gdwarf-2

# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
EXTRAINCDIRS += ../system

# List any extra directories to look for library files here.
#     Each directory must be seperated by a space.
EXTRA_LIBDIRS =

# Compiler flags.
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#
# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
CFLAGS += $(DEBUG)
CFLAGS += $(CDEFS) $(CINCS)
CFLAGS += $(OPT)
CFLAGS += -Wall
#The following are not enabled by -Wall
CFLAGS += -Wcast-align -Wpointer-arith -Wredundant-decls -Wshadow -Wno-attributes
CFLAGS += -fverbose-asm
CFLAGS += -Wa,-adhlns=$(subst $(suffix $<),.s,$<)
CFLAGS += $(patsubst %,-I%,$(call uniq,$(EXTRAINCDIRS)))
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fsingle-precision-constant

# flags only for C++ (arm-elf-g++)
CPPFLAGS = -std=c++14

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map,$(@:.exe=.map),--cref,--gc-sections,--demangle,-v
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS))

CC = gcc
CXX = g++
OBJDUMP=objdump
OBJCOPY=objcopy

# Define all object files.
CPPOBJ    = $(CPPSRC:.cpp=.o)

# Define all listing files.
LST = $(CPPSRC:.cpp=.lst)

# Compiler flags to generate dependency files.
### GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d
GENDEPFLAGS = -MMD -MP -MF .dep/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS  = -I. $(CFLAGS) $(GENDEPFLAGS)

# Default target.
.DEFAULT_GOAL = all
all: build

#ifeq ($(FORMAT),ihex)
#build: elf hex lss sym
#hex: $(TARGET).hex
#IMGEXT=hex
#else 
#ifeq ($(FORMAT),binary)
build: exe lss

exe: $(TARGET).exe
lss: $(TARGET).lss

Doxyfile: ../system/Doxyfile.template
	sed -e 's%^[^I]*INPUT[ \t]*=.*$$%INPUT=$(ALLSRC) $(HDEP) $(INCDEP) $(EXTRADOC)%' \
	    -e 's%^[^P]*PROJECT_NAME[ \t]*=.*$$%PROJECT_NAME="$(TARGET)"%' \
	    -e 's%^[^P]*PROJECT_BRIEF[ \t]*=.*$$%PROJECT_BRIEF=$(PROJECT_BRIEF)%' \
	    < $< > $@

html: Doxyfile $(ALLTAR)
	doxygen

# Create extended listing file from ELF output file.
# testing: option -C
%.lss: %.exe
	$(OBJDUMP) -h $< > $@
	$(OBJDUMP) -p $< >> $@
	$(OBJDUMP) -S -j .text $< | c++filt | tail -n +4 >> $@
	$(OBJDUMP) -s -j .vtable -j .rodata_str -j .rodata -j .ctors $< |c++filt |  tail -n +4 >> $@
	$(OBJCOPY) -O binary -j .source $< $<.tmp.tbz
	if [ -s $<.tmp.tbz ] ;then  echo "Source tarball contents:" >> $@ ; tar tvf $<.tmp.tbz >> $@; fi
	$(RM) -f $<.tmp.tbz 
	$(OBJCOPY) -O binary -j .python $< $<.tmp.py
	if [ -s $<.tmp.py ] ;then  echo "Python contents:" >> $@ ; cat $<.tmp.py >> $@; fi
	$(RM) -f $<.tmp.py
	$(OBJDUMP) -s                        -j .data $< |c++filt |  tail -n +4 >> $@
	$(OBJDUMP) -s -j .ARM.exidx -j .ARM.extab -j .glue -j .vtable_lib -j .rtti_info -j .rtti_name -j .rodata_str_lib -j .rodata_lib -j .ctors_lib $< | tail -n +4 >> $@
	$(OBJDUMP) -t $< | grep ^[0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f] |  sort | c++filt >> $@

ALLOBJ = $(CPPOBJ) $(EXTRAOBJ)

$(TARGET).exe: $(ALLOBJ)
#Rule to link an executable. Whenever another rule specifies a .exe with dependency .o files but no
#recipe, this recipe is used.
%.exe:
	$(CXX) $^ -o $@ $(LDFLAGS)

# Compile: create object files from C++ source files. ARM/Thumb
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
$(CPPOBJ) : %.o : %.cpp
	$(CXX) $(CFLAGS) $(GENDEPFLAGS) $(CPPFLAGS) $(CXXFLAGS) -E $< -o $(@:.o=.e)
	$(CXX) $(CFLAGS) $(CPPFLAGS) $(CXXFLAGS) -Wa,-a,-ad,-aln=$(@:.o=.s) -c $< -o $@

# Target: clean project.
clean:
	@echo
	$(RM) Doxyfile doxygen_sqlite3.db
	$(RM) -r html
	$(RM) -r latex
	$(RM) $(TARGET).exe $(TARGET).map $(TARGET).lss
	$(RM) $(ALLOBJ)
	$(RM) $(EXTRACLEAN)
	$(RM) $(CPPSRC:.cpp=.s) 
	$(RM) $(CPPSRC:.cpp=.d)
	$(RM) $(CPPSRC:.cpp=.e)
	$(RM) .dep/*
	$(RM) doxygen*.tmp

# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(ALLDEP)

PHONY_TGT += all build exe lss clean

# Listing of phony targets.
.PHONY : $(PHONY_TGT)
