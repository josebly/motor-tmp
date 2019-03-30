#!/bin/bash

#dfu-util -s 0x1FFF7800 -a 2 -D parameters/otp.bin
dfu-util -s 0x08060000 -a 0 -R -D parameters/otp.bin

# locking
#cat /dev/zero | head -c 1 > zero.bin
#dfu-util -s 0x1FFF7A00 -a 2 -D zero.bin
#rm zero.bin