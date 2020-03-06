# Tested with linaro toolchain
CC = arm-linux-gnueabihf-gcc
XENO_INSTALL = /home/laur/lucru/raspberry-pi/robertcnelson/ti-linux-kernel-dev/xenomai-userspace-3.0.9

#XENO_CONFIG := /home/laur/lucru/raspberry-pi/robertcnelson/ti-linux-kernel-dev/ignore/xenomai-userspace/usr/xenomai/bin/xeno-config
#/usr/xenomai/bin/xeno-config
#CFLAGS := $(shell $(XENO_CONFIG)   --posix --alchemy --cflags)
#LDFLAGS := $(shell $(XENO_CONFIG)  --posix --alchemy --ldflags)

CFLAGS = -Wall -I$(XENO_INSTALL)/usr/xenomai/include/cobalt -I$(XENO_INSTALL)/usr/xenomai/include/alchemy -I$(XENO_INSTALL)/usr/xenomai/include/smokey
CFLAGS += -I$(XENO_INSTALL)/usr/xenomai/include/trank -I$(XENO_INSTALL)/usr/xenomai/include 
CFLAGS += -mcpu=cortex-a8 -mfpu=neon-vfpv4 -mfloat-abi=hard
#CFLAGS += -march=armv7-a -mfpu=vfpv4 -mfloat-abi=hard
CFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__ -D__COBALT_WRAP_ -fasynchronous-unwind-tables
CFLAGS += -I$(RTNET_INSTALL)/stack/include

LFLAGS = -lalchemy -lcopperplate $(XENO_INSTALL)/usr/xenomai/lib/xenomai/bootstrap.o
LFLAGS += -Wl,--no-as-needed -Wl,@$(XENO_INSTALL)/usr/xenomai/lib/cobalt.wrappers -Wl,@$(XENO_INSTALL)/usr/xenomai/lib/modechk.wrappers 
LFLAGS += -Wl,--wrap=main -Wl,--wrap=malloc -Wl,--wrap=free -Wl,--dynamic-list=$(XENO_INSTALL)/usr/xenomai/lib/dynlist.ld 
LFLAGS += -L$(XENO_INSTALL)/usr/xenomai/lib -lcobalt -lpthread -lrt -lmodechk -lsmokey
LFLAGS += -mcpu=cortex-a8 -mfpu=neon-vfpv4 -mfloat-abi=hard
#LFLAGS +=  -march=armv7-a -mfpu=vfpv4 -mfloat-abi=hard -Llib

LFLAGS_ALCHEMY = -lalchemy -ltrank

all: lat-spi-xenomai.out spitest-laur.out
lat-spi-xenomai.out: lat-spi-xenomai.o
	$(CC) -o lat-spi-xenomai.out lat-spi-xenomai.o $(LFLAGS)
lat-spi-xenomai.o: lat-spi-xenomai.c
	$(CC) -c -o lat-spi-xenomai.o lat-spi-xenomai.c $(CFLAGS)

spitest-laur.out: spitest-laur.o
	$(CC) -o spitest-laur.out spitest-laur.o $(LFLAGS)
spitest-laur.o: spitest-laur.c
	$(CC) -c -o spitest-laur.o spitest-laur.c $(CFLAGS)

.PHONY: clean

clean:
	rm -rf *.o *.so *.o *.out bbio_test
