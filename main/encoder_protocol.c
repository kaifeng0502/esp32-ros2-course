#include <inttypes.h>
#include <stdio.h>
#include "encoder_protocol.h"

int encoder_format_state(char *buffer, size_t size, uint32_t boot_id,
                         int64_t uptime_us, const encoder_snapshot_t *counts)
{
    return snprintf(buffer, size, "ENC2 %" PRIu32 " %" PRId64 " %" PRId64 " %" PRId64 "\n",
                    boot_id, uptime_us, counts->c1_signed, counts->c2_signed);
}
