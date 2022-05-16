obj-m	:= altera_dma_x8.o altera_dma_x4.o

#KERNELDIR ?= /lib/modules/$(shell uname -r)/build
KERNELDIR ?= /usr/src/linux-headers-4.9.140-tegra-ubuntu18.04_aarch64/kernel-4.9
PWD       := $(shell pwd)

CPPFLAGS += -include $(KERNELDIR)/include/generated/autoconf.h
ccflags-y += -Wno-error=date-time

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

install:
	./altera_dma_load

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers *.order

