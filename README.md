![SPL System Architecture](https://s3-us-west-2.amazonaws.com/envirover/images/SPL-2.0.jpg)

[![Build Status](https://travis-ci.org/envirover/SPLRadioRoom.svg?branch=master)](https://travis-ci.org/envirover/SPLRadioRoom)
[![Join the chat at https://gitter.im/SPLRadioRoom/Lobby](https://badges.gitter.im/SPLRadioRoom/Lobby.svg)](https://gitter.im/SPLRadioRoom/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

# SPL RadioRoom

SPL RadioRoom is a firmware for a companion computer of MAVLink-based autopilots such as ArduPilot or PX4 that provides telemetry over Iridium Short Burst Data (ISBD) satellite messaging system. Together with [SPL GroundControl](https://github.com/envirover/SPLGroundControl) server it provides a two-way communication solution between unmanned vehicles and ground control stations such as QGroundControl or Mission Planer.

## System Requirements

SPL RadioRoom system requires the following hardware and software:
* Autopilot such as Pixhawk with [ArduPilot](http://ardupilot.org/) or [PX4](http://px4.io/) firmware;
* Raspberry Pi computer with [Raspbian Stretch](https://www.raspberrypi.org/downloads/raspbian/) Desktop or Light;
* Activated [RockBLOCK Mk2](http://www.rock7mobile.com/products-rockblock) or [RockBLOCK 9603](http://www.rock7mobile.com/products-rockblock-9603) Iridium satellite communication module with FTDI USB to UART cable.

In the minimal configuration RadioRoom requires Raspberry Pi Zero, RockBLOCK 9603, and FTDI chip, with autopilot connected to the Rasperry Pi's serial port. This configuration has the smallest size and power consumption.

The first version of SPL RadioRoom run on Arduino 101. See [arduino/README.md](arduino/README.md) for information about this version.

## Wiring 

SPL RadioRoom uses serial devices to communicate with autopilot and ISBD transceiver. The safest bet is to use USB to TTL UART serial converter to connect both autopilot and transceiver to the Raspberry Pi's USB ports. Another, less straightforward option is to connect autopilot to the Raspberry Pi serial port using GPIO pins. 

![Wiring](https://s3-us-west-2.amazonaws.com/envirover/images/RadioRoomWiring3.jpg)

See the instructions on connecting companion computer to Pixhawk running ArduPilot and PX4 autopilots:
- [ArduPilot Connecting the Pixhawk and RPi](http://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html)
- [PX4 Companion Computer for Pixhawk class](https://dev.px4.io/en/companion_computer/pixhawk_companion.html)

Raspberry Pi was designed to be powered by +5.1V micro USB supply. +5V TELEM pin is rated for up to 2A peak power draw, so it could be used to power both Raspberry Pi and RockBLOCK.  

It is recommended to connect RockBLOCK module using [FTDI USB to UART cable](https://www.rock7.com/shop-product-detail?productId=16) provided by Rock Seven Mobile. 3.3.V FTDI cables or chips from other vendors can also be used, but MaxPower field in the chip must be set to 500 mA to satisfy the power requirements of RockBLOCK (The default setting for MaxPower field is typically 100 mA). Alternatively, RockBLOCK could be powered by directly, not through the FTDI chip.

## Installing

To install RadioRoom on Raspberry Pi:

1. Copy radioroom-2.0.0-raspbian.deb from https://github.com/envirover/SPLRadioRoom/releases to the Raspberry Pi. 
2. Install radioroom-2.0.0-raspbian.deb package.

   ``$ sudo dpkg -i radioroom-2.0.0-raspbian.deb``
  
3. Configure the reporting period and the serial device paths for autopilot and ISBD transceiver in /etc/radioroom.conf. 
4. Start radioroom service.

   ```
   $ sudo systemctl enable radioroom.service
   $ sudo systemctl start radioroom.service
   ```
   
By default the serial device paths are set to /dev/ttyACM0 for autopilot and to /dev/ttyUSB0 for ISBD transceiver. If auto_detect_serials property is set to true, RadioRoom can autodetect autopilot and ISBD if they are available on other serial and USB devices. To make the RadioRoom startup faster and more reliable it is recommended to set the device paths correctly. 

USB device paths /dev/ttyUSB0, /dev/ttyUSB1, ... can swap after reboot. For USB devices it is recommended to use symlinks from /dev/serial/by-path or /dev/serial/by-path directories, that do not change with reboots. 

RadioRoom periodically reports the vehicle's position, attitude, velocity, and other data using HIGH_LATENCY MAVLink message. The message size is 48 bytes. Each report consumes 1 RockBLOCK credit. The reporting period default value is 300 seconds. It can be changed by setting report_period configuration property in /etc/radioroom.conf, or remotely at runtime by setting HL_REPORT_PERIOD on-board parameter.

Raspberry Pi require an orderly shutdown procedure, otherwise the SD card may become corrupted and the system will no longer boot. To prevent the SD card corruption during power cuts it is recommended to [configure Raspbian to work in a read-only mode](https://learn.adafruit.com/read-only-raspberry-pi/). Alternatively, UPS and a shutdown circuit could be used to orderly shutdown Raspberry Pi after power cuts.
  
## Troubleshooting

Run ``$ sudo systemctl status radioroom.service`` to check the status of radioroom service.

If radioroom is properly wired and configured, the output should look like this:

```
pi@raspberrypi:~ $ sudo systemctl status radioroom.service
● radioroom.service - SPL RadioRoom Service
   Loaded: loaded (/etc/systemd/system/radioroom.service; enabled; vendor preset: enabled)
   Active: activating (start) since Tue 2017-11-07 07:27:56 UTC; 6 days ago
     Docs: http://github.com/envirover/SPLRadioRoom
 Main PID: 254 (radioroom)
   CGroup: /system.slice/radioroom.service
           └─254 /usr/sbin/radioroom

Nov 07 07:27:56 raspberrypi systemd[1]: Starting SPL RadioRoom Service...
Nov 07 07:27:57 raspberrypi radioroom[254]: Starting SPL RadioRoom 2.0.0...
Nov 07 07:27:57 raspberrypi radioroom[254]: Connecting to autopilot (/dev/ttyUSB0 57600)...
Nov 07 07:27:58 raspberrypi radioroom[254]: Autopilot detected at serial device '/dev/ttyUSB0'.
Nov 07 07:27:58 raspberrypi radioroom[254]: MAV type: 12, system id: 1, autopilot class: 3, firmware version: 3.5.0/255
Nov 07 07:27:58 raspberrypi radioroom[254]: Connecting to ISBD transceiver (/dev/ttyUSB1 19200)...
Nov 07 07:27:58 raspberrypi radioroom[254]: IRIDIUM 9600 Family SBD Transceiver (IMEA 123456789012345) detected at serial device '/dev/ttyUSB1'.
Nov 07 07:27:58 raspberrypi radioroom[254]: SPL RadioRoom 2.0.0 started.
```

Log file of radioroom service is available at /var/log/radioroom.log.

Add ``-v`` option to the radioroom command line in /etc/systemd/system/radioroom.sevice to enable verbose logging. Verbose logging makes radioroom service log all MAVLink messages sent and received from autopilot and ISBD transceiver.

## Building

To build radioroom on Raspberry Pi. 

```
$ sudo apt-get install git cmake
$ git clone https://github.com/envirover/SPLRadioRoom.git
$ cd SPLRadioRoom
$ mkdir bin
$ cd bin
$ cmake ..
$ make
```

To create a debian package run ``$cpack ..`` command after that.

To cross-compile on Windows.
1. Install git and clone SPLRadioRoom repo.

   ``$ git clone https://github.com/envirover/SPLRadioRoom.git``
   
2. Install [Windows toolchain for Raspberry Pi](http://gnutoolchains.com/raspberry/).
3. Create 'bin' subdirectory inside SPLRadioRoom and change the current directory to it.
4. Run cmake using ../toolchain-arm-windows.cmake toolchain file.

   ``$ cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE:STRING="" -DCMAKE_TOOLCHAIN_FILE:FILEPATH="../toolchain-arm-windows.cmake" ..`` 
5. Run make.

   ``$ make``
   
For cross-compilation on Linux raspbian toolchain for Linux is required. toolchain-arm-linux.cmake should be specified as CMake toolchain file. 

## Issues

Find a bug or want to request a new feature?  Please let us know by submitting an [issue](https://github.com/envirover/SPLRadioRoom/issues).

## Contributing

Envirover welcomes contributions from anyone and everyone. Please see our [guidelines for contributing](https://github.com/envirover/SPLRadioRoom/blob/master/CONTRIBUTING.md).

Licensing
---------
```
Copyright (C) 2017 Envirover

SPL RadioRoom is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

SPL RadioRoom is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with SPLRadioRoom. If not, see <http://www.gnu.org/licenses/>.
```
