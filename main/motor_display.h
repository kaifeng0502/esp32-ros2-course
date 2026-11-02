#pragma once
#include <stdint.h>
#include "motor.h"
#include "encoder.h"

// SH1106 page layout: 128 columns, eight pages of eight pixels.
void motor_display_render(uint8_t frame[1024], const motor_status_t *status,
                          const encoder_snapshot_t *enc, int64_t now_us);
