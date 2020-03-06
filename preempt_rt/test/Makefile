# Tested with linaro toolchain (gcc-linaro-7.4.1-2019.02-x86_64_arm-linux-gnueabihf)
#CC = /home/laur/lucru/raspberry-pi/robertcnelson/ti-linux-kernel-dev/dl/gcc-linaro-7.4.1-2019.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc
CC = arm-linux-gnueabihf-gcc

all: spi-user.out lat-preempt_rt.out
spi-user.out: spi-user.o
	$(CC) -o spi-user.out spi-user.o
spi-user.o: spi-user.c
	$(CC) -c -o spi-user.o spi-user.c
lat-preempt_rt.out: lat-preempt_rt.o
	$(CC) -o lat-preempt_rt.out lat-preempt_rt.o
lat-preempt_rt.o: lat-preempt_rt.c
	$(CC) -c -o lat-preempt_rt.o lat-preempt_rt.c

.PHONY: clean all

clean:
	rm -rf *.out *.so *.o
