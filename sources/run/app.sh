#!/bin/sh
cd ../user_apps
rm user
gcc -pthread user.c i2c.c -o user
./user

