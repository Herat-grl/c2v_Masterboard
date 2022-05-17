#!/bin/sh
make clean
make
insmod phy_intr.ko
gcc user.c -o user
