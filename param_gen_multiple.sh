#!/bin/bash

sn=($(motor-util | tail -n +3 | awk '{print $(NF-1)}'))
path=($(motor-util | tail -n +3 | awk '{print $NF}')) 
echo ${sn[@]}

for (( i=0; i<${#sn[@]}; i++))
do
   ./param_gen --name "J"${sn[$i]}
   dfu-util -s 0x8060000:leave -a0 -D param.bin -p ${path[$i]}
done