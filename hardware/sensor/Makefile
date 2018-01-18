LIBMAKE+=../hardware/sensor/Makefile
CPPSRC+=../hardware/sensor/MPU.cpp
ATTACH+=../hardware/sensor/MPU9250registers.ods
ATTACH+=$(wildcard ../hardware/sensor/*.brd) $(wildcard ../hardware/sensor/*.sch)
ATTACH+=$(wildcard ../hardware/sensor/*.kicad_pcb) $(wildcard ../hardware/sensor/*.pro)
ATTACH+=../hardware/sensor/DownloadDatasheet.sh

EXTRAINCDIRS +=../hardware/sensor/

include ../hardware/Makefile

