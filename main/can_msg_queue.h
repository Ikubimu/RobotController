#ifndef CAN_MSG_QUEUE_H
#define CAN_MSG_QUEUE_H

#include <stdint.h>
#include <time.h>
#include "driver/twai.h"

#define MAX_CAN_MSG 50

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    time_t timestamp;
} can_msg_t;

void can_msg_queue_init(void);
void can_msg_queue_push(const twai_message_t *msg);
int can_msg_queue_get(can_msg_t *out, int max_count);

#endif
