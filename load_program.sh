#!/bin/bash

if ! type motor-util > /dev/null || (( $(motor-util | wc -l) == 2 )) ; then
  dfu-util -s 0x8000000 -a0 -D motor-tmp.bin args $@
  dfu-util -s 0x8060000:leave -a0 -D motor-tmp_param.bin $@
else
 sn=$(motor-util | tail -n +3 | awk '{print $NF}') 
 echo $sn

 echo $sn | xargs -n1 -P10 dfu-util -s 0x8000000 -a0 -D motor-tmp.bin args -p
 echo $sn | xargs -n1 -P10 dfu-util -s 0x8060000:leave -a0 -D motor-tmp_param.bin -p
fi
