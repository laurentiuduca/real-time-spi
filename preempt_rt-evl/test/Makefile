# Tested with linaro toolchain (gcc-linaro-7.4.1-2019.02-x86_64_arm-linux-gnueabihf)
#CC = /home/laur/lucru/raspberry-pi/robertcnelson/ti-linux-kernel-dev/dl/gcc-linaro-7.4.1-2019.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc
#CC = /home/laur/lucru/raspberry-pi/rpi-zero-w/armv6-eabihf--glibc--bleeding-edge-2021.11-1/bin/arm-linux-gcc
CC = arm-linux-gnueabihf-gcc
KERNEL_DIR := /home/laur/lucru/raspberry-pi/rpi-zero-w/linux-6.1.70
#/home/laur/lucru/raspberry-pi/rpi-zero-w/linux-rpi
CFLAGS = -I$(KERNEL_DIR)/include/uapi/evl/devices

all: lat-preempt_rt-evl.out spitest-laur.out
spitest-laur.out: spitest-laur.o
	$(CC) -o spitest-laur.out spitest-laur.o
spitest-laur.o: spitest-laur.c
	$(CC) -c -o spitest-laur.o spitest-laur.c $(CFLAGS)
lat-preempt_rt-evl.out: lat-preempt_rt-evl.o
	$(CC) -o lat-preempt_rt-evl.out lat-preempt_rt-evl.o
lat-preempt_rt-evl.o: lat-preempt_rt-evl.c
	$(CC) -c -o lat-preempt_rt-evl.o lat-preempt_rt-evl.c $(CFLAGS)

.PHONY: clean all

clean:
	rm -rf *.out *.so *.o
