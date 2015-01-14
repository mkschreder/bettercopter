This is source code for firmware for my quadcopter controller. It is built around libk (https://github.com/mkschreder/martink) for maximum portability to any quadcopter control board. 

Building
---------

The build system is currently buildable on linux. For building, you would need to also compile libk. This will require the gcc toolchain that is compiled for your particular hardware. For avr boards you will need avr-gcc and for arm you will need arm-none-eabi-gcc. It is important to use arm-none-eabi and not arm-linux-eabi because we are compiling the firmware to run directly on the hardware without any operating system underneath. 

Install the toolchains (Ubuntu 14.10): 

	apt-get install gcc-arm-none-eabi
	apt-get install gcc-avr

Checkout the submodules: 

	git submodule init
	git submodule update

Now run the configuration utility: 
make menuconfig

You should not need to do much and you can just exit and save the config. This will create a default config for (currently) multiwii v2.5 controller. You can change the config to your liking. This is very similar process to configuring linux kernel for different architechtures - it in fact uses the same utility: menuconfig. 

Now you can build the firmware: 
make

When it is built you can install this on your multiwii (an avr ATMega board) by typing: 
sudo make install_mega

You may need to play with avrdude settings in the Makefile and adjust them to what your defaults for your board are. The default avrdude command uses USBasp to program the board. 

Currently supported hardware
---------------------

Name | Status
MultiWii | Fully supported

Currently supported features
-----------------------

Feature | Description | Status
Stabilized mode | auto level | Supported
Acrobatic mode | direct control | 90 % done
Standard 6 chan RX | radio receiver | Supported
Battery monitor | for lipo | Supported
Mavlink | Standard protocol for communication with ground control | Support for reporting current sensor values, storing parameters in eeprom. 

Quadcopter models
-----------------

Model name | Status
X quad | Supported

Ground control stations
--------------------
Any ground control station that implements mavlink protocol should be supported out of the box. Communication is done over serial interface. 

Ground Control | Status
QGroundControl | Supported
