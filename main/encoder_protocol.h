#pragma once
#include <stddef.h>
#include <stdint.h>
#include "encoder.h"

int encoder_format_state(char *buffer, size_t size, uint32_t boot_id,
                         int64_t uptime_us, const encoder_snapshot_t *counts);
