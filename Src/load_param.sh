#!/bin/bash

dfu-util -s 0x8060000 -a 0 -D param.bin 