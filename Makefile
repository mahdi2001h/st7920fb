ifneq ($(KERNELRELEASE),)
EXTRA_CFLAGS := -I$(PWD)
KBUILD_EXTRA_SYMBOLS := $(PWD)/Module.symvers
obj-m      += st7920fb.o
else
ARCH ?= arm
BUILDROOTDIR ?= $(HOME)/projects/buildroot-sinux
KDIR := $(BUILDROOTDIR)/output/build/linux-5.4.92
CROSS_COMPILE := $(BUILDROOTDIR)/output/host/bin/arm-buildroot-linux-gnueabi-

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$$PWD modules
	
clean: 
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$$PWD clean
	
endif
