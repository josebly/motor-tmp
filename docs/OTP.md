In order to enable the software to know what version of electronic hardware
it is running on, some information is stored in the OTP (one time programmable) 
section of memory. This information is the BoardID struct, which is defined 
in [otp.h](../parameters/otp.h). The program motor_otp_gen is a command line application that is used to help generate the BoardID struct, and saves the file otp.bin. Next one can run motor_load_otp, 
which will upload the otp.bin file as well as write a zero byte to the OTP lock 
memory, which will prevent that block of OTP from being overwritten. 

The otp_gen program provides some help with the -h option. A typical usage of otp_gen 
for an ST Nucleo board is:
```shell
$ motor_otp_gen -s 101
```
For a Fabulab board:
```shell
$ motor_otp_gen -s 101 -m 2 -b 1 -p dev_00
```
Then load:
```shell
$ motor_load_otp
```
