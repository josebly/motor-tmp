#!/usr/bin/env bash

: ${motor_bin:=/usr/share/motor-tmp/motor-tmp.bin}

motors_programmed=0
if ! type motor_util > /dev/null; then
  dfu-util -s 0x8000000:leave -a0 -D ${motor_bin} $@

else
  paths=$(motor_util --list-path-only) 
  echo "${paths}"
  echo "${paths}" | xargs -I{} -n1 -P10 dfu-util -s 0x8000000:leave -a0 -D ${motor_bin} args -p {} && echo "Success"
fi

if (($? == 0)); then
  echo "Motors programmed success"
else
  echo "Error programming motors"
  exit 1
fi
