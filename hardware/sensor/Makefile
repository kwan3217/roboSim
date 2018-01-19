LIBMAKE+=../hardware/sensor/Makefile
CPPSRC+=../hardware/sensor/MPU.cpp
CPPSRC+=../hardware/sensor/BNO055.cpp
ATTACH+=../hardware/sensor/MPU9250registers.ods
ATTACH+=$(wildcard ../hardware/sensor/*.brd) $(wildcard ../hardware/sensor/*.sch)
ATTACH+=$(wildcard ../hardware/sensor/*.kicad_pcb) $(wildcard ../hardware/sensor/*.pro)
ATTACH+=../hardware/sensor/DownloadDatasheet.sh

EXTRAINCDIRS +=../hardware/sensor/

include ../hardware/Makefile
include ../bits/Makefile
include ../math/Makefile

