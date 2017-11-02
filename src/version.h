/*
 * version.h
 *
 *  Created on: Oct 25, 2017
 *      Author: pbobo
 */

#ifndef VERSION_H_
#define VERSION_H_

#define STR(x) #x

#define MAJOR_VERSION 2
#define MINOR_VERSION 0
#ifndef BUILD_NUMBER
    #define BUILD_NUMBER  0
#endif
#define FIRMWARE_VERSION(x, y, z)  STR(x) "." STR(y) "."  STR(z)

#define RADIO_ROOM_VERSION "SPL RadioRoom " FIRMWARE_VERSION(MAJOR_VERSION, MINOR_VERSION, BUILD_NUMBER)

#endif /* VERSION_H_ */
