#!/bin/sh
cd ../kernel_modules/pcie
make clean && make
./install

cd ../gpio
make clean && make
./install.sh

cd ../../user_apps
rm user
gcc -pthread user.c -o user
./user
