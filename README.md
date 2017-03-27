[![Build Status](https://travis-ci.org/envirover/SPLRadioRoom.svg?branch=master)](https://travis-ci.org/envirover/SPLRadioRoom)
[![Join the chat at https://gitter.im/SPLRadioRoom/Lobby](https://badges.gitter.im/SPLRadioRoom/Lobby.svg)](https://gitter.im/SPLRadioRoom/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

# SPLRadioRoom

SPLRadioRoom is an Arduino sketch for a companion computer of ArduPilot-based rovers that enables communication over [RockBLOCK](http://www.rock7mobile.com/products-rockblock) Iridium Short Burst Data satellite communication system (ISBD). Together with [SPLGroundControl](https://github.com/envirover/SPLGroundControl) MAVLink proxy server it provides a two way communication channel between rovers and ground control stations such as MAVProxy, Mission Planer, or QGroundControl.

![SPL System Architecture](https://s3-us-west-2.amazonaws.com/envirover/images/spl.jpg)

SPLRadioRoom reads MAVLink messages from the ArduPilot's telemetry serial port. It integrates high-frequency messages (SYS\_STATUS, GPS\_RAW\_INT, ATTITUDE, GLOBAL\_POSITION\_INT, MISSION\_CURRENT, NAV\_CONTROLLER\_OUTPUT, and VFR\_HUD) into HIGH\_LATENCY message and periodically sends the HIGH\_LATENCY messages to the ISBD. All other messages are encoded and directly forwarded to the ISBD.

Mobile-terminated messages received from the ISBD are decoded and sent directly to the ArduPilot's telemetry serial port.

The default ISBD send-receive session period is 15 minutes (900 seconds). For testing purposes the period could be changed by using "SPLRadioRoom" bluetooth BLE service provided by the sketch. (Eventually it should be changed by a MAVlink command).

SPLRadioRoom requires the following hardware components:
* Arduino 101 board,
* RockBLOCK Mk2 Iridium satellite communication module,
* ArduPilot-based autopilot, such as Pixhawk. 

With additional RS-232 Arduino shield SPLRadioRoom might also work with RockBLOCK+ and RockFLEET modules.

The sketch includes IridiumSBD library code derived from the original [IridiumSBD](https://github.com/mikalhart/IridiumSBD) library.

#### Wiring 

|  Arduino  | RockBLOCK | AP TELEM1 |
|-----------|-----------|-----------|
| +5V       | +5V       | 1 +5V     |
| 2         |           | 2         |
| 3         |           | 3         |
| 8         | TX pin    |           |
| 9         | RX pin    |           |
| 10        | Sleep pin |           |
| Gnd       | Gnd       | 6 Gnd     |

In this wiring diagram both Arduino and RockBLOCK are powered by ArduPilot's TELEM1 power line.

