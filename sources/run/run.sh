#!/bin/sh
cd ../kernel_modules/pcie
make clean && make
./install

cd ../../user_apps
gcc -pthread user.c -o user
./user