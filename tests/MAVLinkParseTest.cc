#include "mavlink.h"
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char** argv) {
    printf("started\n");

    mavlink_high_latency_t  high_latency;
    memset(&high_latency, 0, sizeof(high_latency));
    mavlink_message_t hl_msg;
    mavlink_msg_high_latency_encode(1, 0, &hl_msg, &high_latency);

    uint8_t buf[250];
    int buf_size = mavlink_msg_to_send_buffer(buf, &hl_msg);

   // char str[] = "fe28000100ea1300000000000000000000009f0032fdc83a983aefffffff0000510000000000000001ff00000000394b";
    //  char str[] = "fe28010101ea04000000242c4f14d4f32cbac1fe78fe527d6829d301d4e5010081001700000000090400000000018099";
    /*
    int i = 0;
    while (str[i] != 0) {
        char hex[3];
        hex[0] = str[i];
        hex[1] = str[i+1];
        hex[2] = 0;
        buf[i / 2] = (uint8_t)strtol(hex, NULL, 16);
        i += 2;
    }

    int buf_size = i/2;
    */

    printf("n=%d\n", buf_size);

    mavlink_status_t mavlink_status;
    mavlink_message_t msg;

    for (size_t j = 0; j < buf_size; j++) {
        printf("%2x", buf[j]);

        if (mavlink_parse_char(MAVLINK_COMM_0, buf[j], &msg, &mavlink_status)) {
            printf("\nRECEIVED %d\n", msg.msgid);
            break;
        }
    }

    printf("\n");

    uint8_t buf2[250];
    uint16_t len = mavlink_msg_to_send_buffer(buf2, &msg);

    for (size_t j = 0; j < len; j++) {
        printf("%2x", buf2[j]);
    }

    printf("\ndone\n");
}
