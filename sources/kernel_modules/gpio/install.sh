#!/bin/sh
make clean
make
rmmod phy_intr
insmod phy_intr.ko
