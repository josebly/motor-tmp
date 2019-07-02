# Requirements #
Linux system with an ARM cortex M gcc compiler. On ubuntu this could be installed with apt. However at least one apt version does not seem to work with this project, so I recommend simply getting the latest version from [developer.arm.com](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads). 

Additional dependencies can be automatically installed during the install process and are listed in the [package control file](../DEBIAN/control).

# Recommended #
Instructions and install written assuming an Ubuntu based linux system. Currently most development is done on 18.04.

# Build #
In the root directory run
```
make deb_package
```
This will generate files in the build directory

# Build options #
Additional options can be specified on the make command line
* `LTO=1` provides significant optimization
* `GCC_PATH={PATH}` if the arm gcc compiler is not on your path

# Install #
To install the generated deb package with dependencies:
```
sudo apt install ./motor-tmp*.deb
```
This installs files as described in [INSTALL](INSTALL.md)

# Next steps #
Continue on to [FIRMWARE_DOWNLOAD](FIRMWARE_DOWNLOAD.md)
