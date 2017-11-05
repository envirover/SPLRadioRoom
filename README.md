[![Build Status](https://travis-ci.org/envirover/SPLRadioRoom.svg?branch=master)](https://travis-ci.org/envirover/SPLRadioRoom)
[![Join the chat at https://gitter.im/SPLRadioRoom/Lobby](https://badges.gitter.im/SPLRadioRoom/Lobby.svg)](https://gitter.im/SPLRadioRoom/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

# SPL RadioRoom

SPLRadioRoom is an application for a companion computer of MAVLink-based autopilots such as ArduPilot or PX4 that enables communication over [RockBLOCK](http://www.rock7mobile.com/products-rockblock) Iridium Short Burst Data satellite communication system (ISBD). Together with [SPLGroundControl](https://github.com/envirover/SPLGroundControl) MAVLink proxy server it provides a two way communication channel between rovers and ground control stations such as  QGroundControl, Mission Planer, or MAVProxy.

![SPL System Architecture](https://s3-us-west-2.amazonaws.com/envirover/images/SPL-2.0.jpg)

SPLRadioRoom reads MAVLink messages from the ArduPilot's telemetry serial port. It integrates high-frequency messages (SYS\_STATUS, GPS\_RAW\_INT, ATTITUDE, GLOBAL\_POSITION\_INT, MISSION\_CURRENT, NAV\_CONTROLLER\_OUTPUT, and VFR\_HUD) into HIGH\_LATENCY message and periodically sends the HIGH\_LATENCY messages to the ISBD. All other messages are encoded and directly forwarded to the ISBD.

Mobile-terminated messages received from the ISBD are decoded and sent directly to the autopilot elemetry serial port.

SPLRadioRoom requires the following hardware components:
* Raspberry Pi or Raspberry Pi Zero,
* RockBLOCK Mk2 Iridium satellite communication module,
* ArduPilot-based autopilot, such as Pixhawk. 

SPL RadioRoom includes IridiumSBD library code derived from the original [IridiumSBD](https://github.com/mikalhart/IridiumSBD) library.

#### Wiring 

TBD

## Issues

Find a bug or want to request a new feature?  Please let us know by submitting an [issue](https://github.com/envirover/SPLRadioRoom/issues).

## Contributing

Envirover welcomes contributions from anyone and everyone. Please see our [guidelines for contributing](https://github.com/envirover/SPLRadioRoom/blob/master/CONTRIBUTING.md).

Licensing
---------
```
Copyright (C) 2017 Envirover

SPLGroundControl is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

SPLGroundControl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with SPLRadioRoom. If not, see <http://www.gnu.org/licenses/>.
```
