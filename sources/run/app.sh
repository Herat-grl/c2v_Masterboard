#!/bin/sh
cd ../user_apps
rm user
gcc -pthread user.c -o user
./user

