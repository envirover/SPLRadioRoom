/*
MAVRadioRoom.ino
 
(C) Copyright 2017 Envirover.
 
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
 
#include <CurieBLE.h>
#include <IridiumSBD.h>
#include <SoftwareSerial.h>

#undef F
#include "mavlink/include/standard/mavlink.h"        // Mavlink interface

#ifndef M_PI
#define M_PI 3.14159265359
#endif

#define ISBD_MAX_MT_MGS_SIZE 270

#define SYSTEM_ID    1
#define COMPONENT_ID MAV_COMP_ID_UART_BRIDGE

// Pixhawk telemetry interface
#define AP_TELEM_RX_PIN    2  //connect to Pixhawk TELEM1 pin 2 
#define AP_TELEM_TX_PIN    3  //connect to Pixhawk TELEM1 pin 3 

//iridium SBD transciever interface
#define ISBD_RX_PIN        8  //connect to ISBD TX pin
#define ISBD_TX_PIN        9  //connect to ISBD RX pin
#define ISBD_SLEEP_PIN     10 //connect to ISBD sleep pin

#define LED_PIN            13

#define AP_TELEM_BAUD_RATE 57600
#define ISBD_BAUD_RATE     19200

// Default HIGH_LATENCY message reporting period in milliseconds
#define HL_MSG_REPORT_PERIOD 10000L

#define BLE_SERVICE_NAME   "MAVRadioRoom"


SoftwareSerial ardupilot(AP_TELEM_RX_PIN, AP_TELEM_TX_PIN);

SoftwareSerial nss(ISBD_RX_PIN, ISBD_TX_PIN);
IridiumSBD isbd(nss, ISBD_SLEEP_PIN);

mavlink_message_t mavlink_message; 
mavlink_status_t mavlink_status;
mavlink_high_latency_t high_latency_msg;
unsigned long last_report_time;
uint8_t high_latency_seq = 0;

// Bluetooth service 
BLEPeripheral ble_peripheral; 
BLEService ble_service("861c0e0a-ded3-11e6-bf01-fe55135034f3"); 
BLEUnsignedLongCharacteristic ble_high_latency_period("a4cf3be2-ded3-11e6-bf01-fe55135034f3", BLERead | BLEWrite);

void setup() 
{
  	Serial.begin(57600);
    
    ardupilot.begin(AP_TELEM_BAUD_RATE);
    pinMode(LED_PIN, OUTPUT);
    last_report_time = 0;

    // Init high_latency_msg
    memset(&high_latency_msg, 0, sizeof(high_latency_msg));
    high_latency_msg.gps_nsat = 255;
    high_latency_msg.landed_state = MAV_LANDED_STATE_UNDEFINED;
  
      // set the local name peripheral advertises
    ble_peripheral.setLocalName(BLE_SERVICE_NAME);
    // set the UUID for the service this peripheral advertises:
    ble_peripheral.setAdvertisedServiceUuid(ble_service.uuid());
  
    // add service and characteristics
    ble_peripheral.addAttribute(ble_service);
    ble_peripheral.addAttribute(ble_high_latency_period);
  
    ble_high_latency_period.setValue(HL_MSG_REPORT_PERIOD);
  
    // advertise the service
    ble_peripheral.begin();

    // Init SBD
    nss.begin(ISBD_BAUD_RATE);

    isbd.attachConsole(Serial);
    isbd.attachDiags(Serial);
    isbd.setPowerProfile(1);
    isbd.begin();

    int signalQuality = -1;
    int err = isbd.getSignalQuality(signalQuality);
    if (err != 0) {
        Serial.print("SignalQuality failed: error ");
        Serial.println(err);
    } else {
        Serial.print("SignalQuality: ");
        Serial.println(signalQuality);
    }
}

void loop() 
{ 
    ble_peripheral.poll();
    
  	// Send the message (.write sends as bytes) 
  	//ardupliot.write(buf, len);
  	
  	comm_receive();

    unsigned long current_time = millis();

    if (current_time - last_report_time > ble_high_latency_period.value()) {
        print_high_latency_msg(high_latency_msg);
        
        mavlink_message_t mavlink_msg; 
        mavlink_msg_high_latency_encode(SYSTEM_ID, COMPONENT_ID, &mavlink_msg, &high_latency_msg);
        mavlink_msg.seq = high_latency_seq++;

        isbd_session(mavlink_msg);                

        last_report_time = current_time;
    }
}

inline int16_t rad_to_centidegrees(float rad) 
{
    return rad / M_PI * 18000;
}

/*
 * Integrates high frequency message into HIGH_LATENCY type message.
 * 
 * @param high_latency meggase of MavLink message of HIGH_LATENCY type
 * @param msg message received from Ardupilot
 * @return true if the message was integrated or should be just swallowed
 */
bool update_high_latency_msg(mavlink_high_latency_t& high_latency, const mavlink_message_t msg) 
{
    switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT://0
            high_latency.base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
            high_latency.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
            return true;
        case MAVLINK_MSG_ID_SYS_STATUS://1
            high_latency_msg.battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg);
            return true;
        case MAVLINK_MSG_ID_SYSTEM_TIME://2
            return true;
        case MAVLINK_MSG_ID_GPS_RAW_INT://24
            high_latency.gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
            high_latency.gps_nsat = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
            return true;
        case MAVLINK_MSG_ID_RAW_IMU://27
            return true;
        case MAVLINK_MSG_ID_ATTITUDE://30
            high_latency.roll = rad_to_centidegrees(mavlink_msg_attitude_get_roll(&msg));
            high_latency.pitch = rad_to_centidegrees(mavlink_msg_attitude_get_pitch(&msg));
            return true;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT://33
            high_latency.latitude = mavlink_msg_global_position_int_get_lat(&msg);
            high_latency.longitude = mavlink_msg_global_position_int_get_lon(&msg);
            high_latency.altitude_amsl = mavlink_msg_global_position_int_get_alt(&msg);
            high_latency.altitude_sp = mavlink_msg_global_position_int_get_relative_alt(&msg);
            return true;
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW://35
            return true;
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW://36
            return true;
        case MAVLINK_MSG_ID_MISSION_CURRENT://42
            high_latency.wp_num = mavlink_msg_mission_current_get_seq(&msg);
            return true;
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT://62
            high_latency.wp_distance = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
            high_latency_msg.heading_sp = mavlink_msg_nav_controller_output_get_nav_bearing(&msg) * 100;
            return true;
        case MAVLINK_MSG_ID_VFR_HUD://74
            high_latency.airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
            high_latency.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
            high_latency.heading = mavlink_msg_vfr_hud_get_heading(&msg) * 100;
            high_latency.climb_rate = mavlink_msg_vfr_hud_get_climb(&msg);
            high_latency.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
            return true;
        case MAVLINK_MSG_ID_SCALED_IMU2://116
            return true;
        case MAVLINK_MSG_ID_POWER_STATUS://125
            return true;
        case MAVLINK_MSG_ID_VIBRATION://241
            return true;
    }
  
    return false;
}

// Debug print of HIGH_LATENCY message
void print_high_latency_msg(const mavlink_high_latency_t& high_latency_msg) 
{
    Serial.println("**");
    Serial.print("custom_mode = "); Serial.println(high_latency_msg.custom_mode);
    Serial.print("latitude = "); Serial.println(high_latency_msg.latitude);
    Serial.print("longitude = "); Serial.println(high_latency_msg.longitude);
    Serial.print("roll = "); Serial.println(high_latency_msg.roll);
    Serial.print("pitch = "); Serial.println(high_latency_msg.pitch);
    Serial.print("heading = "); Serial.println(high_latency_msg.heading);
    Serial.print("heading_sp = "); Serial.println(high_latency_msg.heading_sp);
    Serial.print("altitude_amsl = "); Serial.println(high_latency_msg.altitude_amsl);
    Serial.print("altitude_sp = "); Serial.println(high_latency_msg.altitude_sp);
    Serial.print("wp_distance = "); Serial.println(high_latency_msg.wp_distance);
    Serial.print("base_mode = "); Serial.println(high_latency_msg.base_mode);
    Serial.print("landed_state = "); Serial.println(high_latency_msg.landed_state);
    Serial.print("throttle = "); Serial.println(high_latency_msg.throttle);
    Serial.print("airspeed = "); Serial.println(high_latency_msg.airspeed);
    Serial.print("airspeed_sp = "); Serial.println(high_latency_msg.airspeed_sp);
    Serial.print("groundspeed = "); Serial.println(high_latency_msg.groundspeed);
    Serial.print("climb_rate = "); Serial.println(high_latency_msg.climb_rate);
    Serial.print("gps_nsat = "); Serial.println(high_latency_msg.gps_nsat);
    Serial.print("gps_fix_type = "); Serial.println(high_latency_msg.gps_fix_type);
    Serial.print("battery_remaining = "); Serial.println(high_latency_msg.battery_remaining);
    Serial.print("temperature = "); Serial.println(high_latency_msg.temperature);
    Serial.print("temperature_air = "); Serial.println(high_latency_msg.temperature_air);
    Serial.print("failsafe = "); Serial.println(high_latency_msg.failsafe);
    Serial.print("wp_num = "); Serial.println(high_latency_msg.wp_num);
}

/*
 * Debug print of mavlink_message_t message
 */
void print_mavlink_msg(const mavlink_message_t& msg) 
{
    Serial.println("**");
    Serial.print("msgid = "); Serial.println(msg.msgid);
    Serial.print("compid = "); Serial.println(msg.compid);
}

/*
 * Send the message to ISBD, recieve all the messages in the 
 * inbound message queue, if any, and pass them to ardupilot.
 */
void isbd_session(const mavlink_message_t& msg) 
{
    uint8_t buf[ISBD_MAX_MT_MGS_SIZE];
    size_t  buf_size = sizeof(buf);
    
    //Copy the message to send buffer 
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    do {
        nss.listen();
        if (isbd.sendReceiveSBDBinary(buf, len, buf, buf_size) == ISBD_SUCCESS) {
            len = 0;
            if (buf_size > 0) {
                ardupilot.listen();
                ardupilot.write(buf, buf_size);      

                delay(100);
                
                while (ardupilot.available() > 0) { 
                    uint8_t c = ardupilot.read();
                      
                    // Try to get a new message 
                    if (mavlink_parse_char(MAVLINK_COMM_0, c, &mavlink_message, &mavlink_status)) { 
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                    } 
                }
            }
        }
    } while (isbd.getWaitingMessageCount() > 0 || len > 0);
}

/*
 * Reads and processes MavLink messages from ArduPilot.
 */
void comm_receive() 
{ 
  	// Receive data from Ardupilot
    ardupilot.listen();
    
  	while (ardupilot.available() > 0) { 
        digitalWrite(LED_PIN, HIGH);
    		
    		uint8_t c = ardupilot.read();
    		
    		// Try to get a new message 
    		if (mavlink_parse_char(MAVLINK_COMM_0, c, &mavlink_message, &mavlink_status)) { 
      			// Handle message
            if (!update_high_latency_msg(high_latency_msg, mavlink_message)) {
                print_mavlink_msg(mavlink_message);
                
                isbd_session(mavlink_message);
            } 
    		} 
  	}
  
    digitalWrite(LED_PIN, LOW);
}

/*
bool ISBDCallback()
{
   comm_receive();
   
   return true;
}
*/
