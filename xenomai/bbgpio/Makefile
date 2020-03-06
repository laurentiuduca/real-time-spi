
obj-m += xeno_bbgpio.o
obj-m += xeno_osc-gpio-rtdm.o

xeno_bbgpio-y := bbgpio.o
xeno_osc-gpio-rtdm-y := osc-gpio-rtdm.o

INSTALL_DIR ?= ./
PWD ?= $(shell pwd)
KERNEL_DIR := /home/laur/lucru/raspberry-pi/robertcnelson/ti-linux-kernel-dev/ipipe-greg

ccflags-y += -I$(KERNEL_DIR)/include/xenomai

ARCH		= arm
# launch command: 
# make CROSS_COMPILE="/home/laur/lucru/raspberry-pi/robertcnelson/ti-linux-kernel-dev/dl/gcc-linaro-7.4.1-2019.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-"
AS              = $(CROSS_COMPILE)as
LD              = $(CROSS_COMPILE)ld
CC              = $(CROSS_COMPILE)gcc
CPP             = $(CC) -E
AR              = $(CROSS_COMPILE)ar
NM              = $(CROSS_COMPILE)nm
STRIP           = $(CROSS_COMPILE)strip
OBJCOPY         = $(CROSS_COMPILE)objcopy
OBJDUMP         = $(CROSS_COMPILE)objdump

export ARCH AS LD CC
export CPP AR NM STRIP OBJCOPY OBJDUMP

.PHONY: all build clean

#all: build info
all: build

build:
	make -C $(KERNEL_DIR) M=$(PWD) modules

#info:
#	modinfo $(PWD)/xeno_bbgpio.ko

clean:
	@make -C $(KERNEL_DIR) M=$(PWD) clean
