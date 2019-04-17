#!/bin/bash

dfu-util -s 0x8000000 -a0 -D build/motor-tmp.bin
dfu-util -s 0x8060000 -a0 -D build/motor-tmp_param.bin 