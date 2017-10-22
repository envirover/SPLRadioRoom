/*
 * MAVLinkLogger.cpp
 *
 *  Created on: Oct 22, 2017
 *      Author: Pavel Bobov
 */

#include <stdio.h>
#include "MAVLinkLogger.h"

#define MAX_LOG_MESSAGE_SIZE 1024

void MAVLinkLogger::log(int priority, const char* prefix,
        const mavlink_message_t& message) {
    char buff[1024];

    switch (message.msgid) {
    case MAVLINK_MSG_ID_HIGH_LATENCY:
        mavlink_high_latency_t high_latency;
        mavlink_msg_high_latency_decode(&message, &high_latency);
        snprintf(buff, sizeof(buff), "%s msgid=%d, sysid=%d, compid=%d, seq=%d, "
                "custom_mode=%d, latitude=%d, longitude=%d, roll=%d, pitch=%d, heading=%d, "
                "heading_sp=%d, altitude_amsl=%d, altitude_sp=%d, wp_distance=%d, base_mode=%d, "
                "landed_state=%d, throttle=%d, airspeed=%d, airspeed_sp=%d, groundspeed=%d, "
                "climb_rate=%d, gps_nsat=%d, gps_fix_type=%d, battery_remaining=%d, "
                "temperature=%d, temperature_air=%d",
                prefix, message.msgid, message.sysid, message.compid,
                message.seq, high_latency.custom_mode, high_latency.latitude,
                high_latency.longitude, high_latency.roll, high_latency.pitch,
                high_latency.heading, high_latency.heading_sp,
                high_latency.altitude_amsl, high_latency.altitude_sp,
                high_latency.wp_distance, high_latency.base_mode,
                high_latency.landed_state, high_latency.throttle,
                high_latency.airspeed, high_latency.airspeed_sp,
                high_latency.groundspeed, high_latency.climb_rate,
                high_latency.gps_nsat, high_latency.gps_fix_type,
                high_latency.battery_remaining, high_latency.temperature,
                high_latency.temperature_air);
        break;
    default:
        snprintf(buff, sizeof(buff), "%s msgid=%d, sysid=%d, compid=%d, seq=%d",
                prefix, message.msgid, message.sysid, message.compid,
                message.seq);
        break;
    }

    syslog(priority, buff);
}
