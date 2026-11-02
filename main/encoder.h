#pragma once

#include <stdint.h>

#define ENCODER_COUNTS_PER_REV 1400.0f

typedef struct {
    uint32_t c1_edges;
    uint32_t c2_edges;
    int64_t c1_signed;
    int64_t c2_signed;
    int c1_level;
    int c2_level;
} encoder_snapshot_t;

void encoder_init(void);
encoder_snapshot_t encoder_snapshot(void);
