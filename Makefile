ifneq ($(KERNELRELEASE),)
EXTRA_CFLAGS := -I$(PWD)
KBUILD_EXTRA_SYMBOLS := $(PWD)/Module.symvers
obj-m      += st7920fb.o
else
ARCH ?= arm
CCPREFIX ?= /home/dsv/RaspberryPi/tools/arm-bcm2708/arm-bcm2708hardfp-linux-gnueabi/bin/arm-bcm2708hardfp-linux-gnueabi- 
CROSS_COMPILE ?= $(CCPREFIX)

KDIR ?= /home/dsv/RaspberryPi/rpi-buildroot/output/build/linux-rpi-4.0.y

#CONFIG_DEBUG_SECTION_MISMATCH=y

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$$PWD modules
	
clean: 
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$$PWD clean
	
endif