LIBMAKE+=../hardware/piInterface/Makefile
CPPSRC+=../hardware/piInterface/HardwarePi.cpp
ATTACH+=../hardware/piInterface/DownloadDatasheet.sh $(wildcard ../hardware/piInterface/*.brd) $(wildcard ../hardware/piInterface/*.sch)

EXTRAINCDIRS +=../hardware/piInterface/
EXTRA_LIBS+=wiringPi

include ../hardware/sensor/Makefile
include ../interface/Makefile
include ../bits/Makefile


