obj-m += phy_intr.o
 
#KDIR = /lib/modules/$(shell uname -r)/build
KDIR = /usr/src/linux-headers-4.9.140-tegra-ubuntu18.04_aarch64/kernel-4.9
#KDIR = /home/prasanna/GRL_C2V/Build/sources/kernel/kernel-4.9/tegra_image 
 
all:
	make ARCH=arm64 -C $(KDIR)  M=$(shell pwd) modules
 
clean:
	make ARCH=arm64 -C $(KDIR)  M=$(shell pwd) clean
