If this is the first programming of the device ever, set bootloader mode by setting jumpers and power cycling. If not the first programming just continue with device plugged in.

For the first programming start by loading the one time programmable memory as described in [OTP.md].

Next load the code by running `./load_program.sh` from its directory

Finally, to change parameters use `./param_gen` and `./load_param.sh`. 

Help for `param_gen` and `otp_gen` is available with the `-h` option.

