After building firmware.
Generate .dfu file
python hex2dfu.py build/usb_vcp_ze.hex 
If this is the first programming of the device, set bootloader mode by setting jumpers and power cycling. If not the first programming just continue with device plugged in.
dfu-util -a0 -D build/usb_vcp_ze.dfu
