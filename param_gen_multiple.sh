#!/usr/bin/env bash

names=("")
for name in $names
do
    ./build/param_gen --config ini/r0_gimb2.ini --name $name
    mv param.bin param_$name.bin
    echo ""
done