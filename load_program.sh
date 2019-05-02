#!/bin/bash

sn=$(motor-util | tail -n +3 | awk '{print $NF}') 
echo $sn

#dfu-util -s 0x8000000 -a0 -D build/motor-tmp.bin args $@
#dfu-util -s 0x8060000:leave -a0 -D build/motor-tmp_param.bin $@
echo $sn | xargs -n1 -P10 dfu-util -s 0x8000000 -a0 -D build/motor-tmp.bin args -p
echo $sn | xargs -n1 -P10 dfu-util -s 0x8060000:leave -a0 -D build/motor-tmp_param.bin -p