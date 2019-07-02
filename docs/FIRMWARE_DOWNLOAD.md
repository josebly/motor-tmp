If this is the first programming of the device ever, set bootloader mode by setting jumpers and power cycling. If not the first programming just continue with device plugged in.

For the first programming start by loading the one time programmable memory as described in [OTP](OTP.md).

Next load the code by running `motor_load_program`.

Finally, to change parameters use `motor_param_gen`, which generates a param.bin file in the current directory and `motor_load_param` to load that param.bin file. 

Help for `motor_param_gen` and `motor_otp_gen` is available with the `-h` option.

