#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "lwip/sockets.h"

typedef struct {
    uint32_t sequence;
    uint8_t pixels[1024];
    struct sockaddr_in sender;
} network_frame_t;

void network_init(void);
bool network_command(char *line);
bool network_take_frame(network_frame_t *frame);
void network_ack(const network_frame_t *frame);
