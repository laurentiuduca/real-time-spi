# Tested with linaro toolchain (gcc-linaro-7.4.1-2019.02-x86_64_arm-linux-gnueabihf)
CC = arm-linux-gnueabihf-gcc
EVL_INSTALL = /home/laur/lucru/raspberry-pi/evl/evl
KERNEL_DIR = /home/laur/lucru/raspberry-pi/evl/linux-evl

CFLAGS = -Wall -I$(EVL_INSTALL)/include -I$(KERNEL_DIR)/include/uapi/evl/devices
CFLAGS += -mcpu=cortex-a8 -mfpu=neon-vfpv4 -mfloat-abi=hard
#CFLAGS += -march=armv7-a -mfpu=vfpv4 -mfloat-abi=hard
CFLAGS += -D_GNU_SOURCE -D_REENTRANT
CFLAGS += -fasynchronous-unwind-tables

#LFLAGS += -Wl,--no-as-needed -Wl,--wrap=malloc -Wl,--wrap=free
LFLAGS += -L$(EVL_INSTALL)/lib -levl -leshi -lpthread -lrt
LFLAGS += -mcpu=cortex-a8 -mfpu=neon-vfpv4 -mfloat-abi=hard

all: spitest-evl.out lat-bbb-evl.out
spitest-evl.out: spitest-evl.o
	$(CC) -o spitest-evl.out spitest-evl.o $(LFLAGS)
spitest-evl.o: spitest-evl.c
	$(CC) -c -o spitest-evl.o spitest-evl.c $(CFLAGS)

lat-bbb-evl.out: lat-bbb-evl.o
	$(CC) -o lat-bbb-evl.out lat-bbb-evl.o $(LFLAGS)
lat-bbb-evl.o: lat-bbb-evl.c
	$(CC) -c -o lat-bbb-evl.o lat-bbb-evl.c $(CFLAGS)

.PHONY: clean all

clean:
	rm -rf *.out *.so *.o
