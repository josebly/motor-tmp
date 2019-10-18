#!/usr/bin/env bash

names=("$@")
for name in $names
do
    echo "Loading $name param"
    dfu-util -s 0x8060000:leave -a 0 -D param_$name.bin -p $(motor_util -n $name --list-path-only)
    if [ $? ]; then
        echo "Error loading param for $name"
        exit $?
    fi
done
