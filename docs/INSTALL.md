The currently recommended install process is to build the deb_package and install with `apt` or `dpkg`. Files are installed to the operating system as follows:
* `/usr/bin/` scripts prefixed with `motor_` that are useful for downloading firmware and setting parameters
* `/usr/share/motor-tmp/` the firmware file `motor-tmp.bin` and a default parameter file `motor-tmp_param.bin`
* `/usr/share/motor-tmp/ini/` configuration files for various hardware systems that are used with the `motor_param_gen` program
* `/usr/share/motor-tmp/docs/` this documentation
* `/etc/udev/rules.d/99-st.rules` allows for non root access to the device. The package will run `udevadm control --reload` after install to update. Currently plugged in usb devices will need to be unplugged and replugged to recognize updated rules.