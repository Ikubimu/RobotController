#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "can_msg_queue.h"

static can_msg_t s_buffer[MAX_CAN_MSG];
static int s_head = 0;
static int s_count = 0;
static SemaphoreHandle_t s_mutex;

void can_msg_queue_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
}

void can_msg_queue_push(const twai_message_t *msg)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    int idx = (s_head + s_count) % MAX_CAN_MSG;

    s_buffer[idx].id = msg->identifier;
    s_buffer[idx].dlc = msg->data_length_code;
    memcpy(s_buffer[idx].data, msg->data, msg->data_length_code);
    time(&s_buffer[idx].timestamp);

    if (s_count < MAX_CAN_MSG) {
        s_count++;
    } else {
        s_head = (s_head + 1) % MAX_CAN_MSG;
    }

    xSemaphoreGive(s_mutex);
}

int can_msg_queue_get(can_msg_t *out, int max_count)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    int to_copy = s_count < max_count ? s_count : max_count;
    for (int i = 0; i < to_copy; i++) {
        int idx = (s_head + s_count - 1 - i) % MAX_CAN_MSG;
        memcpy(&out[i], &s_buffer[idx], sizeof(can_msg_t));
    }

    xSemaphoreGive(s_mutex);
    return to_copy;
}
