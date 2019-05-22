#!/bin/bash

: ${motor_bin:=/usr/share/motor-tmp/motor-tmp.bin}
: ${motor_param_bin:=/usr/share/motor-tmp/motor-tmp_param.bin}

if ! type motor-util > /dev/null || (( $(motor-util | wc -l) == 2 )) ; then
  dfu-util -s 0x8000000:leave -a0 -D ${motor_bin} $@
  if [ ! -z $motor_param_bin ]; then
    dfu-util -s 0x8060000:leave -a0 -D ${motor_param_bin} $@
  fi
else
 sn=$(motor-util | tail -n +3 | awk '{print $NF}') 
 echo $sn

 echo $sn | xargs -n1 -P10 dfu-util -s 0x8000000:leave -a0 -D ${motor_bin} args -p
 if [ ! -z $motor_param_bin ]; then
  echo $sn | xargs -n1 -P10 dfu-util -s 0x8060000:leave -a0 -D ${motor_param_bin} -p
 fi
fi
